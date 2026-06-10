package org.firstinspires.ftc.teamcode.teleop.opModes.test.servoTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(group = "tests")
public class StopperTest extends OpMode {
    Robot robot;
    public static double stopperPos = 0.6; // release 0.48
    public static double stage2Speed = 0;
    GamepadMapping controls;
    ServoImplEx stopper;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        stopper = hardwareMap.get(ServoImplEx.class, "stopper");

    }

    @Override
    public void loop() {
//        robot.stopper.stopper.setPosition(stopperPos);
        if (gamepad1.b) {
            stopper.setPosition(0.8);
        } else {
            stopper.setPosition(1);
        }
    }
}
