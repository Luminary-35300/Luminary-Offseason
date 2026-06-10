package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {

    // could be Bore encoder OR internal axon encoder
    public AnalogInput analogInput1;
    public AnalogInput analogInput2;

    public Servo turret1;
    public Servo turret2;

    private PIDController controller;
    private double p = 0, i = 0, d = 0; //has to be tuned
    private double f = 0;

    public Turret(HardwareMap hwMap) {
        turret1 = hwMap.get(Servo.class, "turret1");
        turret2 = hwMap.get(Servo.class, "turret2");
        analogInput1 = hwMap.get(AnalogInput.class, "analogEncoder1");
        analogInput2 = hwMap.get(AnalogInput.class, "analogEncoder2");

        this.controller = new PIDController(p, i, d);
    }

    public double powerToPosition(double power) {
        return power * 2 - 1;
    }

    public void setPower(double power) {
        // Use if doing CRServo I believe
        // turret1.setPower(power);
        // turret2.setPower(power);
    }

    // TODO may have to make 2 run to pos, one for each servo
    public void runToPos(double targetAngle) {
        controller.setPID(p, i, d);
        // If using bore
        // double pos = this.getBorePosition();
        double pos = this.getServoPosition();
        double pid = controller.calculate(pos, targetAngle);
        double ff = Math.cos(Math.toRadians(targetAngle)) * f;
        double power = pid + ff;

        turret1.setPosition(powerToPosition(power));
        turret2.setPosition(powerToPosition(power));
    }

    public double getServoPosition() {
        return analogInput1.getVoltage() / 3.3 * 360;
    }

    // in degrees
    public double getBorePosition() {
        double temp = analogInput1.getVoltage() / 3.3 * 360;
        return angleWrap(temp) / 2048;
    }

    public void changePIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    private double angleWrap(double angle) {
        angle = Math.toRadians(angle);
        // Changes any angle between [-179,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * (angle > 0 ? 1 : -1); // if angle > 0 * 1, < 0 * -1
        }
        return Math.toDegrees(angle);
    }
}
