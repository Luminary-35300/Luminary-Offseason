package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Limelight {

    private final Limelight3A limelight;

    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // PD tuning
    public static double kP                      = 0.036;
    public static double kD                      = 0.0008;
    public static double goalX                   = 0.0;
    public static double angleTolerance          = 0.4;

    // Distance calibration
    public static double REF_DISTANCE_INCHES     = 126.0;
    public static double TARGET_AREA_AT_REF_DIST = 0.32;

    private double lastError = 0;
    private double lastTime  = 0;

    private LLResult lastResult = null;
    private boolean wasAligned = false;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront .setDirection(DcMotor.Direction.REVERSE);
        leftBack  .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lastTime = System.currentTimeMillis();
    }


    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /** Call once per loop() to refresh the latest result and update IMU orientation. */
    public void update(YawPitchRollAngles orientation) {
        limelight.updateRobotOrientation(orientation.getYaw());
        lastResult = limelight.getLatestResult();
    }

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    public double getTargetArtifactTravelDistanceX() {
        return getDistanceInches() + 22;
    }

    public boolean hasTarget() {
        return lastResult != null && lastResult.isValid();
    }

    /** Horizontal bearing to target in degrees (tX). Positive = target is to the right. */
    public double getAngleBearing() {
        return hasTarget() ? lastResult.getTx() : 0.0;
    }

    /** Target area as % of image (tA). */
    public double getTargetArea() {
        return hasTarget() ? lastResult.getTa() : 0.0;
    }

    /** Estimated distance to target in inches using inverse-square law. Returns -1 if no target. */
    public double getDistanceInches() {
        if (!hasTarget()) return -1;
        double tA = lastResult.getTa();
        return (tA > 0) ? REF_DISTANCE_INCHES * Math.sqrt(TARGET_AREA_AT_REF_DIST / tA) : -1;
    }

    /** True if bearing error is within angleTolerance. */
    public boolean isAligned() {
        return hasTarget() && Math.abs(getAngleBearing() - goalX) <= angleTolerance;
    }

    // -------------------------------------------------------------------------
    // Drive methods
    // -------------------------------------------------------------------------

    /**
     * Full mecanum drive with current limiting — call this every loop for normal driver control.
     * axial   = forward/back  (left stick Y, negated)
     * lateral = strafe        (left stick X)
     * yaw     = rotation      (right stick X)
     */
    public void drive(double axial, double lateral, double yaw) {
        mecanumDrive(axial, lateral, yaw);
    }

    /**
     * Auto-align yaw toward the target while still allowing driver axial/lateral input.
     * Runs at full power (no current limiting) so the correction is never throttled.
     * Returns true if aligned, false if still correcting (or no target).
     */
    public boolean autoAlign(double axial, double lateral, Gamepad gamepad) {
        if (!hasTarget()) {
            mecanumDriveAutoAlign(axial, lateral, 0);
            lastError = 0;
            wasAligned = false;
            return false;
        }

        double curTime = System.currentTimeMillis();
        double dt      = (curTime - lastTime) / 1000.0;
        lastTime = curTime;

        double error      = getAngleBearing() - goalX;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        double yawPower   = kP * error + kD * derivative;
        lastError = error;

        if (Math.abs(error) > angleTolerance) {
            // FIX: use mecanumDriveAutoAlign so current limiting never throttles alignment
            mecanumDriveAutoAlign(axial, lateral, yawPower);
            wasAligned = false;
            return false;
        } else {
            mecanumDriveAutoAlign(axial, lateral, 0);

            if (Math.abs(error) < 1.7) {
                gamepad.rumble(0.8, 0.8, 400);
            }

            return true;
        }
    }

    /** Stop all drive motors. */
    public void stopDrive() {
        mecanumDrive(0, 0, 0);
    }

    // -------------------------------------------------------------------------
    // Internal
    // -------------------------------------------------------------------------

    /** Current-limited drive — used for normal driver control. */
    private void mecanumDrive(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        double lfCurrent = leftFront.getCurrent(CurrentUnit.AMPS);
        double rfCurrent = rightFront.getCurrent(CurrentUnit.AMPS);
        double lbCurrent = leftBack.getCurrent(CurrentUnit.AMPS);
        double rbCurrent = rightBack.getCurrent(CurrentUnit.AMPS);

        double averageCurrent = (lfCurrent + rfCurrent + lbCurrent + rbCurrent) / 4;
        if (averageCurrent > 6) {
            lf *= 0.5;
            rf *= 0.5;
            lb *= 0.5;
            rb *= 0.5;
        }

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }

    /** Full-power drive (no current limiting) — used for auto-align. */
    private void mecanumDriveAutoAlign(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }
}