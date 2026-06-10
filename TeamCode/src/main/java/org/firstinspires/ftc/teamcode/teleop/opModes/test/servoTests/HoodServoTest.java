package org.firstinspires.ftc.teamcode.teleop.opModes.test.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp (group = "tests")
public class HoodServoTest extends OpMode {
    //Robot robot;

    // HOOD SERVO RANGE
    // .65 is all the way down
    // .05 is all the way up

    public static double servoPos = 0;
    public static double power = 0;
    GamepadMapping controls;
    private Servo variableHood;
    private Robot robot;
    private Intake intake;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        intake = new Intake(hardwareMap, controls);
        variableHood = hardwareMap.get(Servo.class, "variableHood");
    }

    @Override
    public void loop() {
//        intakeTransfer.intakeTransferOn();
//        controls.update();
//        //robot.drivetrain.update();
//
//        robot.shooter.setShooterVelocity(-power);
//        //robot.shooter.setShooterPower(-power);
        variableHood.setPosition(servoPos);

        // right bumper
        //if (controls.transfer.value()) {
            //robot.transfer.transferOn();
//        telemetry.addData("Current shooter vel",robot.shooter.outtake1.getVelocity());
//        telemetry.update();
        //}
        // left bumper
//        if (controls.intakeTransfer.value()) {
//            robot.intakeTransfer.intakeOn();
//        }
    }
}
