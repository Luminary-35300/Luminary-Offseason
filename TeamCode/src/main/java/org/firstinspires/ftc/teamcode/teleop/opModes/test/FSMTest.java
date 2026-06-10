package org.firstinspires.ftc.teamcode.teleop.opModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

@Config
@TeleOp(name="FSMTest", group="test")
public class FSMTest extends OpMode {
    // Standard Robot Classes
    private Robot robot;
    private GamepadMapping controls;
    private FSM fsm;

    // Pedro Drivetrain
    private Follower follower;
    private Shooter shooter;
    public static double shooterVelocity = 0.6;
    public static double hoodAngle = 0.5;

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);
        shooter = new Shooter(hardwareMap);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        fsm.update();

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x * 0.55;

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        telemetry.addData("Control Type", fsm.getControlType());
        telemetry.addData("Switch value", controls.switchMode.value());
        telemetry.addData("Current Transfer Draw (Amps)", robot.intake.getTransferCurrent());

        //shooter.outtake1.setPower(1.0);
        //shooter.outtake2.setPower(1.0);
        //shooter.hoodToFront();
    }

    @Override
    public void stop() {
        PoseStorage.startingPose = follower.getPose();
    }
}
