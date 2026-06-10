package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

import dev.frozenmilk.sinister.util.flag.Test;

@Config
@TeleOp(name ="0Final TeleOp", group = "tests")
public class TeleOpTest extends OpMode {
    Robot robot;
    GamepadMapping controls;
    ServoImplEx stopper;
    TestFSM fsm;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        stopper = hardwareMap.get(ServoImplEx.class, "stopper");
        fsm = new TestFSM(hardwareMap,controls, robot);

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

    }

    @Override
    public void start() {
        Robot.limelight.start();
    }

    @Override
    public void loop() {
        if (!gamepad1.right_bumper) {
            stopper.setPosition(0.8);
        } else {
            stopper.setPosition(1);
        }

        fsm.update();

        // Keep limelight fresh every loop
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        Robot.limelight.update(orientation);

        double forward = -Math.pow(gamepad1.left_stick_y, 3);
        double strafe  = Math.pow(gamepad1.left_stick_x, 3);
        double rotate  = Math.pow(gamepad1.right_stick_x*0.92, 3);

        if (gamepad1.right_trigger > 0) {
            // FIX: autoAlign() already drives the motors internally (full power, no current limit).
            // Do NOT call mecanumDriveAutoAlign() here as well — that would overwrite the yaw
            // correction that autoAlign just applied, breaking the alignment entirely.
            boolean aligned = Robot.limelight.autoAlign(forward, strafe, gamepad1);

            telemetry.addLine(">> AUTO ALIGNING (hold A)");
            telemetry.addData("aligned",           aligned);
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
            telemetry.addData("has target",        Robot.limelight.hasTarget());
        } else {
            // Full driver control with current limiting
            mecanumDrive(forward, strafe, rotate);

            telemetry.addLine(">> DRIVER CONTROL (hold A to align)");
            telemetry.addData("has target",        Robot.limelight.hasTarget());
            telemetry.addData("bearing (tX)",      Robot.limelight.getAngleBearing());
            telemetry.addData("distance (inches)", Robot.limelight.getDistanceInches());
        }
        telemetry.addData("ol", robot.shooter.outtake1.getVelocity());
        telemetry.addData("or", robot.shooter.outtake2.getVelocity());
        telemetry.addData("hood pos", robot.shooter.variableHood.getPosition());

        telemetry.addData("Control Type", fsm.getControlType());
        telemetry.addData("Switch value", controls.switchMode.value());
        telemetry.addData("Total current",robot.shooter.outtake1.getCurrent(CurrentUnit.AMPS)+robot.shooter.outtake2.getCurrent(CurrentUnit.AMPS)+robot.intake.dropdownIntake.getCurrent(CurrentUnit.AMPS)+robot.intake.transfer.getCurrent(CurrentUnit.AMPS)+leftFront.getCurrent(CurrentUnit.AMPS)+leftBack.getCurrent(CurrentUnit.AMPS)+rightFront.getCurrent(CurrentUnit.AMPS)+rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.limelight.stop();
    }

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
            lf *= 0.7;
            rf *= 0.7;
            lb *= 0.7;
            rb *= 0.7;
        } else {
            lf *= 1.0;
            rf *= 1.0;
            lb *= 1.0;
            rb *= 1.0;
        }

        leftFront .setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack  .setPower(lb / max);
        rightBack .setPower(rb / max);
    }
}
