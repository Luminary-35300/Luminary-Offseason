package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name = "Auto Align Test Ishaan", group = "test")
public class AutoAlignTest_Ishaan extends OpMode {

    private Limelight limelight;
    private Robot              robot;
    private GamepadMapping     controls;
    public static int shooterVel = 900;
    public static double hoodAngle = 0.5;


    @Override
    public void init() {
        controls  = new GamepadMapping(gamepad1, gamepad2);
        robot     = new Robot(hardwareMap, controls);
        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // Update limelight with current IMU orientation
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        limelight.update(orientation);

        // Driver inputs
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x * 0.55;

        if (gamepad1.a) {
            // Hold A: auto-align yaw, driver keeps axial/lateral control
            boolean aligned = limelight.autoAlign(axial, lateral, gamepad1);

            telemetry.addLine(">> AUTO ALIGNING (hold A)");
            telemetry.addData("aligned",           aligned);
            telemetry.addData("bearing (tX)",      limelight.getAngleBearing());
            telemetry.addData("distance (inches)", limelight.getDistanceInches());
            telemetry.addData("tA (raw)",          limelight.getTargetArea());

            if (!limelight.hasTarget()) {
                telemetry.addLine("   WARNING: No tag visible");
            }
        } else {
            // Full driver control
            limelight.drive(axial, lateral, yaw);
            telemetry.addLine(">> DRIVER CONTROL (hold A to align)");
            telemetry.addData("has target",        limelight.hasTarget());
            telemetry.addData("bearing (tX)",      limelight.getAngleBearing());
            telemetry.addData("distance (inches)", limelight.getDistanceInches());
        }

        telemetry.update();
        robot.shooter.setShooterVelocity(shooterVel);
        robot.intake.intakeTransferOnClose();
        robot.shooter.setHoodAngle(hoodAngle);
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}