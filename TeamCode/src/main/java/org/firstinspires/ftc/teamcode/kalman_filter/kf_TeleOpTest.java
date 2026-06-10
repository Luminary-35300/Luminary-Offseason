package org.firstinspires.ftc.teamcode.kalman_filter;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp(name = "KF TeleOp Test", group = "Kalman Filter")
public class kf_TeleOpTest extends OpMode {
    public static double INCHES_PER_METER = 39.3701;

    // Kalman Filter Tuning Parameters
    // Q: Process noise covariance (trust in odometry). Lower = trust more.
    // R: Measurement noise covariance (trust in Limelight). Lower = trust more.
    public static double Q = 0.1;
    public static double R = 0.4;

    private Limelight3A limelight;
    private Follower follower;

    private KalmanFilter kfX, kfY, kfHeading;
    private Pose lastFollowerPose;
    private Pose filteredPose = new Pose();
    private Pose limelightPose = new Pose();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        // Initialize 1D Kalman Filters for X, Y, and Heading
        kfX = new KalmanFilter(Q, R);
        kfY = new KalmanFilter(Q, R);
        kfHeading = new KalmanFilter(Q, R);

        lastFollowerPose = follower.getPose();
        
        // Seed the filter with the initial odometry pose
        kfX.setState(lastFollowerPose.getX());
        kfY.setState(lastFollowerPose.getY());
        kfHeading.setState(lastFollowerPose.getHeading());
    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentFollowerPose = follower.getPose();

        // 1. Prediction Step: Use odometry change (deltas) as the control input
        double deltaX = currentFollowerPose.getX() - lastFollowerPose.getX();
        double deltaY = currentFollowerPose.getY() - lastFollowerPose.getY();
        double deltaHeading = currentFollowerPose.getHeading() - lastFollowerPose.getHeading();

        // Normalize heading delta to [-pi, pi]
        deltaHeading = Math.atan2(Math.sin(deltaHeading), Math.cos(deltaHeading));

        kfX.predict(deltaX);
        kfY.predict(deltaY);
        kfHeading.predict(deltaHeading);

        lastFollowerPose = currentFollowerPose;

        // 2. Update Step: Use Limelight measurement if a valid target is visible
        Pose cameraEstimate = getRobotPoseFromCamera();
        if (cameraEstimate != null) {
            limelightPose = cameraEstimate;
            
            kfX.update(cameraEstimate.getX());
            kfY.update(cameraEstimate.getY());

            // Handle heading wrap-around during update
            double headingError = cameraEstimate.getHeading() - kfHeading.getState();
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            
            // Correct the state by updating with (current state + error)
            kfHeading.update(kfHeading.getState() + headingError);
        }

        // 3. Final Filtered Pose: Combine states
        filteredPose = new Pose(kfX.getState(), kfY.getState(), kfHeading.getState());

        // Update telemetry
        telemetry.addData("Follower Pose (Odo)", currentFollowerPose.toString());
        telemetry.addData("Limelight Pose (Cam)", cameraEstimate != null ? cameraEstimate.toString() : "No Target");
        telemetry.addData("Filtered Pose (Fused)", filteredPose.toString());
        telemetry.update();

        // Drive Code
        double forward = -Math.pow(gamepad1.left_stick_y, 3);
        double strafe  = Math.pow(gamepad1.left_stick_x, 3);
        double rotate  = Math.pow(gamepad1.right_stick_x, 3);
        follower.setTeleOpDrive(forward, strafe, rotate);
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private Pose getRobotPoseFromCamera() {
        // Feed the current follower heading to Limelight for more accurate BotPose calculations
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getBotpose() == null) return null;

        Pose3D botpose = result.getBotpose();

        // Convert Limelight's meters to inches and transform to Pedro Pathing coordinates
        return new Pose(
                botpose.getPosition().x * INCHES_PER_METER,
                botpose.getPosition().y * INCHES_PER_METER,
                botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE
        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
