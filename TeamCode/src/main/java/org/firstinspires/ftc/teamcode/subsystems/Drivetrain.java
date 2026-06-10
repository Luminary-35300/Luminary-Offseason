package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

public class Drivetrain {
    // HARDWARE (I (Ishaan) added Final when changing from pedro)
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final IMU imu;

    // MISC
    private GamepadMapping controls;

    private double newX = 0;
    private double newY = 0;

    private DriveMode driveMode;

    public Drivetrain(HardwareMap hardwareMap, IMU imu, GamepadMapping controls){
        // TODO: Configure these properly
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        this.imu = imu;

        this.controls = controls;

        driveMode = DriveMode.ROBO_CENTRIC;
    }

    public Drivetrain(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack, IMU imu) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.imu = imu;
    }

    public void update() {
        double strafe = controls.strafe;
        double drive = controls.drive;
        double turn = controls.turn;

        if (driveMode.equals(DriveMode.FIELD_CENTRIC)) {
            moveFieldCentric(strafe, drive, turn, getHeading());
        } else if (driveMode.equals(DriveMode.ROBO_CENTRIC)){
            moveRoboCentric(-0, drive, -turn);
        }
    }

    public void moveRoboCentric(double strafe, double drive, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        strafe = Math.tan(strafe);
        drive = Math.tan(drive);
        turn = Math.tan(turn);

        leftFront.setPower((drive + strafe + turn) / denominator);
        leftBack.setPower((drive - strafe + turn) / denominator);
        rightFront.setPower((drive - strafe - turn) / denominator);
        rightBack.setPower((drive + strafe - turn) / denominator);
    }

    public void moveFieldCentric(double inX, double inY, double turn, double currentAngle){
        currentAngle += 90;
        double radian = Math.toRadians(currentAngle);
        double cosTheta = Math.cos(radian);
        double sinTheta = Math.sin(radian);
        newX = (-inX * sinTheta) - (inY * cosTheta);
        newY = (-inX * cosTheta) + (inY * sinTheta);

        moveRoboCentric(newX,newY,-turn);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double angleWrap(double angle) {
        angle = Math.toRadians(angle);
        // Changes any angle between [-179,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * (angle > 0 ? 1 : -1); // if angle > 0 * 1, < 0 * -1
        }
        return Math.toDegrees(angle);
    }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBO_CENTRIC
    }
}
