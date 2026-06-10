package org.firstinspires.ftc.teamcode.subsystems.turret;


public class TurretMath {

    public static void main(String[] args) {


        double xt = 0.0, yt = 0.0;     // turret position (get from odo)
        double xT = 0.0, yT = 4.0;     // target position (hard code)
        double headingDeg = 0.0;      // turret’s current heading (get from encoder value and convert to angle)

        double dthetaDeg = TurretMath.requiredRotationDeg(xt, yt, xT, yT, headingDeg);

        System.out.printf("Rotate turret by %.2f degrees%n", dthetaDeg);
    }

    // Normalize any angle to (-pi, pi] /
    public static double wrapToPi(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /** Compute the absolute angle from turret to target (radians) */
    public static double absoluteBearing(double xt, double yt,
                                         double xT, double yT) {
        // vector to target
        return Math.atan2(yT - yt, xT - xt);
    }

    /** Compute how much to rotate turret (radians) */
    public static double requiredRotation(double xt, double yt,
                                          double xT, double yT,
                                          double headingRad) {
        double dx = xT - xt;   // X distance to target
        double dy = yT - yt;   // Y distance to target

        if (dx == 0 && dy == 0) {
            // turret and target in same spot, direction undefined
            return 0.0;
        }

        double theta = Math.atan2(dy, dx);    // angle to target
        double dtheta = theta - headingRad;   // how far off we are
        return wrapToPi(dtheta);              // shortest rotation
    }

    /** Same thing but in degrees */
    public static double requiredRotationDeg(double xt, double yt,
                                             double xT, double yT,
                                             double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        double dthetaRad = requiredRotation(xt, yt, xT, yT, headingRad);
        return Math.toDegrees(dthetaRad);
    }
}
