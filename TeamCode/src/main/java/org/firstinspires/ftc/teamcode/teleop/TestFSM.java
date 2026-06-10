package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


public class TestFSM {
    // --------------- Robot & States ---------------
    public Robot robot;
    public TestFSMStates state = TestFSMStates.BASE_STATE;
    public ControlType type = ControlType.PID_CONTROL;
    private final GamepadMapping gamepad;

    // --------------- SUBSYSTEMS ---------------
    private final Intake intake;
    private final Shooter shooter;

    // --------------- MISC ---------------
    public double lastVelo = 1350; //1200
    public double lastHoodPos = 0.94;
    private ControlType savedType;

    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;
    private double manualOffset  = 0.0;

    private static final double HOOD_MIN        = 0;
    private static final double HOOD_MAX        = 0.65;
    private static double VELOCITY_CORRECTION = 35;


    public TestFSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        shooter = robot.shooter;

        savedType = ControlType.PID_CONTROL;
    }

    public void update() {
        gamepad.update();

        switch (state) {
            case BASE_STATE:

                // Intake toggle

                if (gamepad.gamepad2.a) {
                    intake.intakeTransferOff();
                    intake.pivotUp();
                } else if (gamepad.gamepad2.y) {
                    intake.intakeTransferOnClose();
                    intake.pivotDown();
                } else if (gamepad.intakeTransfer.value()) {
                    intake.intakeTransferOnClose();
                    intake.pivotDown();
                } else if (gamepad.intakeOnly.locked()) {
                    intake.dropdownIntake.setPower(1.0);
                    intake.pivotDown();
                } else if (!gamepad.transfer.locked() || (!gamepad.intakeOnly.locked())) {
                    intake.intakeTransferOff();
                    intake.pivotUp();
                }

                // Stopper hold (PID mode only)
                if (gamepad.transfer.locked() && type == ControlType.PID_CONTROL) {
                    intake.intakeTransferOnClose();
                    intake.pivotDown();
                } else if (type == ControlType.PID_CONTROL) {


                }

                // Outtake hold
                if (gamepad.outtake.locked()) {
                    state = TestFSMStates.OUTTAKING;
                }

                // --------------- Shared dpad manual offset (both modes) ---------------
                boolean dpadUpPressed   = gamepad.gamepad2.dpad_up   && !lastDpadUp;
                boolean dpadDownPressed = gamepad.gamepad2.dpad_down && !lastDpadDown;

                if (dpadUpPressed)   manualOffset += 0.05;
                if (dpadDownPressed) manualOffset -= 0.05;

                lastDpadUp   = gamepad.gamepad2.dpad_up;
                lastDpadDown = gamepad.gamepad2.dpad_down;

                // --------------- PID Only ---------------
                if (type == ControlType.PID_CONTROL) {

                    double[][] velocityPoints = {
                            {45,   1250},
                            {54,   1340},
                            {65,   1380},
                            {82,   1410},
                            {100,  1600}

                    };

                    double[][] hoodPoints = {
                            {45,  0.56},
                            {54,  0.57},
                            {65,  0.59},
                            {82,  0.59},
                            {100,  0.59}

                    };

                    double velocitySlope     = calcSlope(velocityPoints);
                    double velocityIntercept = calcIntercept(velocityPoints, velocitySlope);
                    double hoodSlope         = calcSlope(hoodPoints);
                    double hoodIntercept     = calcIntercept(hoodPoints, hoodSlope);

                    double distance       = Robot.limelight.getDistanceInches();
                    double targetVelocity = (velocitySlope * distance + velocityIntercept);
                    double targetHoodPos  = hoodSlope     * distance + hoodIntercept;

                    // --- Limelight dropout protection ---
                    if (distance != -1) {
                        lastVelo    = targetVelocity;
                        lastHoodPos = targetHoodPos;
                    }

                    // --- Clamp hood position with manual offset ---
                    double clampedHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, lastHoodPos + manualOffset));

                    if (Robot.limelight.getTargetArtifactTravelDistanceX() == 22) {
                        robot.shooter.setHoodAngle(clampedHood);
                        robot.shooter.setShooterVelocity(lastVelo + 300 + VELOCITY_CORRECTION);
                    } else {
                        robot.shooter.setHoodAngle(clampedHood);
                        robot.shooter.setShooterVelocity(lastVelo + VELOCITY_CORRECTION);
                    }
                }

                // --------------- Mode Switch ---------------
                if (gamepad.switchMode.value()) {
                    if (savedType == ControlType.PID_CONTROL) {
                        type = ControlType.HARDCODED_CONTROL;
                        savedType = ControlType.HARDCODED_CONTROL;
                        gamepad.switchMode.set(false);
                    } else if (savedType == ControlType.HARDCODED_CONTROL) {
                        type = ControlType.PID_CONTROL;
                        savedType = ControlType.PID_CONTROL;
                        gamepad.switchMode.set(false);
                    }
                }

                // --------------- Hardcoded Only ---------------
                if (type == ControlType.HARDCODED_CONTROL) {
                    double clampedHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, lastHoodPos + manualOffset));
                    shooter.shootHardcoded();
                    robot.shooter.hoodToBack();

                    // Shoot on right bumper hold
                    if (gamepad.gamepad1.right_bumper) {
                        intake.intakeTransferOnClose();
                        intake.pivotDown();
                    } else {
                        intake.pivotUp();
                    }
                }

                if (gamepad.shootBack.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = TestFSMStates.HARDCODED;
                }

                break;

            case OUTTAKING:
                robot.intake.intakeTransferReverse();

                if (!gamepad.outtake.locked()) {
                    state = TestFSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }
                break;

            case HARDCODED:
                shooter.shootHardcoded();
                shooter.hoodToBack();
                intake.intakeTransferOnClose();

                if (!gamepad.shootBack.locked()) {
                    state = TestFSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }
                break;
        }
    }

    public void setState(TestFSMStates newState) {
        state = newState;
    }

    public TestFSMStates getState() {
        return state;
    }

    public void setControlType(ControlType newCType) {
        type = newCType;
    }

    public ControlType getControlType() {
        return type;
    }

    public enum TestFSMStates {
        BASE_STATE,
        HARDCODED,
        OUTTAKING
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

    private double calcSlope(double[][] points) {
        int n = points.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (double[] p : points) {
            sumX  += p[0];
            sumY  += p[1];
            sumXY += p[0] * p[1];
            sumX2 += p[0] * p[0];
        }
        return (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    }

    private double calcIntercept(double[][] points, double slope) {
        int n = points.length;
        double sumX = 0, sumY = 0;
        for (double[] p : points) { sumX += p[0]; sumY += p[1]; }
        return (sumY - slope * sumX) / n;
    }
}