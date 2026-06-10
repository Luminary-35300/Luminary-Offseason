package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


@Disabled
@Config
@TeleOp(name ="TeleOp Test", group = "tests")
public class TeleOpTest extends OpMode {
    Limelight limelight;
    IMU imu;

    private Follower follower;



    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.update();
    }

    @Override
    public void start() {
        limelight.start();

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        //Update Limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.update(orientation);

        //Drive Code
        double forward = -Math.pow(gamepad1.left_stick_y, 3);
        double strafe  = Math.pow(gamepad1.left_stick_x, 3);
        double rotate  = Math.pow(gamepad1.right_stick_x, 3);
        follower.setTeleOpDrive(forward,strafe,rotate);
    }

    @Override
    public void stop() {
        limelight.stop();
    }

}
