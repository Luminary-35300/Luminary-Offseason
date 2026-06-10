package org.firstinspires.ftc.teamcode.kalman_filter;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class localization_TeleOp extends OpMode {
    public static double INCHES_PER_METER = 39.3701;

    private Limelight3A limelight;
    private Follower follower;

    // This variable stores the robot's pose as estimated by the Limelight
    private Pose limelightPose = new Pose();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

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

        // Update the limelightPose variable from limelight data
        Pose estimate = getRobotPoseFromCamera();
        if (estimate != null) {
            limelightPose = estimate;
        }

        telemetry.addData("Follower Pose", follower.getPose().toString());
        telemetry.addData("Limelight Pose", limelightPose.toString());
        telemetry.update();

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

    private Pose getRobotPoseFromCamera() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getBotpose() == null) return null;

        Pose3D botpose = result.getBotpose();

        return new Pose(
                botpose.getPosition().x * INCHES_PER_METER,
                botpose.getPosition().y * INCHES_PER_METER,
                botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE
        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
