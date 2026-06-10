package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

public class Robot {
// CONFIG
    // right - expansion
    // left - control
    // front - 0
    // back - 1

    // servo variableHood    control hub 5
    // servo stopper         control hub 2

    // servo park            expansion hub 5

    // motor outtakeTop      control hub 2
    // motor outtakeBot      control hub 3

    // motor intake1         expansion hub port _
    // motor dropDownIntake         expansion hub port _

    // led                   control hub 6-7

    // beambreaks            ---------------

    // MECHANISMS

    public final IMU imu;
    public Intake intake;
//    public Stopper stopper;
    public Shooter shooter;
    public Drivetrain drivetrain;

    //public GoBildaPinpointDriver driver;

    public GamepadMapping controls;

    public static Limelight limelight;

    public DigitalChannel ledBoard0;
    public DigitalChannel ledBoard1;

    public Robot(HardwareMap hardwareMap, GamepadMapping controls) {
        this.controls = controls;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        //driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = new Limelight(hardwareMap);

        intake = new Intake(hardwareMap, controls);
//        stopper = new Stopper(hardwareMap);
        shooter = new Shooter(hardwareMap);


        drivetrain = new Drivetrain(hardwareMap, imu, controls);
    }

    public Robot(GamepadMapping controls, IMU imu, GoBildaPinpointDriver pinpoint,
                 Intake intake, Stopper stopper, Shooter shooter, Drivetrain dt,
                 DigitalChannel led0, DigitalChannel led1) {
        this.controls = controls;
        this.imu = imu;
        //this.driver = pinpoint;
        this.intake = intake;
//        this.stopper = stopper;
        this.shooter = shooter;
        this.drivetrain = dt;
        this.ledBoard0 = led0;
        this.ledBoard1 = led1;
    }
}
