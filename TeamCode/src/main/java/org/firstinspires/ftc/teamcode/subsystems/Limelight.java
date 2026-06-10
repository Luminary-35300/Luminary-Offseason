package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Limelight {
    private final Limelight3A limelight;


    // Distance calibration
    public static double REF_DISTANCE_INCHES     = 126.0;
    public static double TARGET_AREA_AT_REF_DIST = 0.32;

    private double lastError = 0;
    private double lastTime  = 0;

    private LLResult lastResult = null;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        lastTime = System.currentTimeMillis();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /** Call once per loop() to refresh the latest result and update IMU orientation. */
    public void update(YawPitchRollAngles orientation) {
        limelight.updateRobotOrientation(orientation.getYaw());
        lastResult = limelight.getLatestResult();
    }


    public boolean hasTarget() {
        return lastResult != null && lastResult.isValid();
    }

    /** Horizontal bearing to target in degrees (tX). Positive = target is to the right. */
    public double getAngleBearing() {
        return hasTarget() ? lastResult.getTx() : 0.0;
    }

    /** Target area as % of image (tA). */
    public double getTargetArea() {
        return hasTarget() ? lastResult.getTa() : 0.0;
    }

    /** Estimated distance to target in inches using inverse-square law. Returns -1 if no target. */
    public double getDistanceInches() {
        if (!hasTarget()) return -1;
        double tA = lastResult.getTa();
        return (tA > 0) ? REF_DISTANCE_INCHES * Math.sqrt(TARGET_AREA_AT_REF_DIST / tA) : -1;
    }


}