package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public final DcMotorEx outtake1;
    public final DcMotorEx outtake2;
    // hood fully up is 0
    // hood fully down is 0.65
    public final Servo variableHood;

    public Shooter(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "ol"); //outtake left
        outtake2 = hardwareMap.get(DcMotorEx.class, "or"); //outtake right
        outtake1.setVelocityPIDFCoefficients(1000, 0, 0, 12);
        outtake2.setVelocityPIDFCoefficients(1000, 0, 0, 12);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        variableHood = hardwareMap.get(Servo.class, "variableHood");
    }

    public Shooter(DcMotorEx outtake1, DcMotorEx outtake2, Servo hood) {
        this.outtake1 = outtake1;
        this.outtake2 = outtake2;
        this.variableHood = hood;
    }

    public enum outtakeVels {
        // 5.059
        HARDCODED_SHOOT_FRONT(1300),
        // 5.954
        HARDCODED_SHOOT_BACK(1400),
        HARDCODED(1350),
        IDLE(0);

        private final double outtake_vels;

        outtakeVels(double pos) {
            this.outtake_vels = pos;
        }

        public double getOuttakeVel() {
            return outtake_vels;
        }
    }

    //-----------------Math-----------------\\
    private static final double g = 9.79284;
    // y distance to goal - to the middle of the shooter
    private static final double H = 0.360837;
    private static double shootVel;

    // calculates target velocity with GIVEN distance in INCHES from the goal
    public static double calculateShooterMPS(double d) {
        double R = d * 0.0254;
        shootVel = Math.sqrt(H * g + g * Math.sqrt(Math.pow(R, 2) + Math.pow(H, 2)));
        return shootVel;
    }

    // calculates target velocity with CURRENT distance away from the goal
    public double calculateShooterMPS() {
        return calculateShooterRPM(Robot.limelight.getTargetArtifactTravelDistanceX());
    }

    // calculates target velocity in TICKS PER SECOND instead of meters per second
    public double calculateShooterRPM(double d) {
        return convertMPSToRPM(calculateShooterMPS(d));
    }

    // converts the target velocity from meters per second to rpm for DcMotor
    public static double convertMPSToRPM(double mpsVel) {
        double c = -0.947834;
        double a = 12.70476;
        double b = 0.000686427;
        double lnArgument = Math.abs(1.0 - ((mpsVel - c) / a));
        return -Math.log(lnArgument) / b; // Math.log is natural logarithm
    }

    // HOOD ANGLE CALCULATIONS
    // ---------------------------------
    private static double hoodAngle;

    // returns the target angle in RADIANS depending on GIVEN distance in INCHES from the april tag
    public static double calculateHoodAngle(double d) {
        double R = d * 0.0254;
        hoodAngle = Math.atan(Math.pow(calculateShooterMPS(d), 2) / (g * R));
        return hoodAngle;
    }

    // returns the target angle in RADIANS depending on CURRENT distance from the april tag
    public double calculateHoodAngle() {
        return calculateHoodAngle(Robot.limelight.getTargetArtifactTravelDistanceX());
    }

    // returns the target angle in HOOD POS (0-1) instead of radians
    public double calculateHoodPos(double d) {
        return convertTargetAngleToHoodPos(calculateHoodAngle(d));
    }

    // converts the target angle from calculateHoodAngle() to a servo position from 0-1
    public static double convertTargetAngleToHoodPos(double targetAngle) {
        double m = 59.72666;
        double b = 24.98837;
        return (Math.toDegrees(targetAngle) - b) / m;
    }

    public static double getShootVel() {
        return shootVel;
    }


//-------------------------------------------------------------------------------

    public void setShooterVelocity(double velo) {
        outtake1.setVelocity(velo);
        outtake2.setVelocity(velo);
    }

    public void setHoodAngle(double angle) {
        variableHood.setPosition(angle);
    }

    // fully down is .65
    // fully up is 0
    public void hoodToBack() {
        variableHood.setPosition(0.4);
    }

    public void hoodToFront() {
        variableHood.setPosition(0.05);
    }

    public void shootFromBack() {
        outtake1.setVelocity(outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel());
        outtake2.setVelocity(outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel());
    }

    public void shootFromFront() {
        outtake1.setVelocity(outtakeVels.HARDCODED_SHOOT_FRONT.getOuttakeVel());
        outtake2.setVelocity(outtakeVels.HARDCODED_SHOOT_FRONT.getOuttakeVel());
    }

    public void shootHardcoded() {
        outtake1.setVelocity(outtakeVels.HARDCODED.getOuttakeVel());
        outtake2.setVelocity(outtakeVels.HARDCODED.getOuttakeVel());
    }

}



