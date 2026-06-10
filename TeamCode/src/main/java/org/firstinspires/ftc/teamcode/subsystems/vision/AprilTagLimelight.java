package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Objects;
import java.util.stream.Stream;

import dev.nextftc.hardware.impl.IMUExKt;


public class AprilTagLimelight {
    Limelight3A limelight;
    boolean isBlue;

    public AprilTagLimelight(HardwareMap hw, boolean isBlue, Telemetry t) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        this.isBlue = isBlue;


        t.addLine(String.format("connected: %s, running: %s, status: %s, index: %s",
                limelight.isConnected(),
                limelight.isRunning(),
                limelight.getStatus(),
                limelight.getStatus().getPipelineIndex())
        );
    }
    public double getDistance(IMU imu){
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            FiducialResult fresult = result.getFiducialResults().get(0);
            return Math.sqrt(Math.pow(fresult.getRobotPoseTargetSpace().getPosition().y, 2) +
                    Math.pow(fresult.getRobotPoseTargetSpace().getPosition().x,2));
        }
        return 0.0;
    }
    public double getAngle(IMU imu){
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            FiducialResult fresult = result.getFiducialResults().get(0);
            return fresult.getTargetXDegrees();

        }
        return 0.0;
    }




    private boolean correctAT(int id){
        return (id == 20 && isBlue) || (id == 24 && !isBlue);
    }
    public enum ObeliskLocation //measured by the location of the green
    {
        LEFT("GPP", 21), CENTER("PGP", 22), RIGHT("PPG", 23);

        public final String order;
        public final int ATnumber;
        ObeliskLocation(String o, int AT){
            order = o;
            ATnumber = AT;
        }

        public static ObeliskLocation fromInt(int i){
            for(ObeliskLocation ol : ObeliskLocation.values()){
                if (ol.ATnumber == i)
                    return ol;
            }
            return null;
        }
    }

}
