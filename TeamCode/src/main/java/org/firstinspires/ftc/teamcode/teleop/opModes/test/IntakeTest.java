package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp (name = "Intake Test", group = "tests")
public class IntakeTest extends OpMode {

    public static double power = 1.0;
    public static double position = 0.72;
    private Intake intake;
    DcMotor leftFront, rightFront;
    private GamepadMapping controls;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        intake = new Intake(hardwareMap, controls);
    }

    @Override
    public void loop() {
        intake.setPivotPos(position);
        intake.setIntakePower(power);
    }
}
