package fsmTests.mathTests;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@ExtendWith(MockitoExtension.class)
public class DynamicShootingTests {

    // 4 TESTS

    // -- SHOOTER --
    @Mock
    DcMotorEx outtake1, outtake2;

    @Mock
    Servo hood;

    Shooter shooter;

    @Mock
    logi webCam;

    // -- INTAKE --
    @Mock
    DcMotorEx intake1;
    @Mock
    DcMotorEx intake2;

    // -- STOPPER --
    @Mock
    Servo stopperServo;

    // -- PARK --
    @Mock
    Servo park1;
    @Mock
    Servo park2;

    // -- DRIVETRAIN --
    @Mock
    DcMotorEx leftFront;
    @Mock
    DcMotorEx rightFront;
    @Mock
    DcMotorEx leftBack;
    @Mock
    DcMotorEx rightBack;
    @Mock
    IMU imu;

    // -- OTHER HARDWARE --
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    @Mock
    // uh this prob wont run so use the interface or spy it if you need it
    GoBildaPinpointDriver pinpoint;

    @Mock
    DigitalChannel led0, led1;


    // -- ACTUAL OBJECTS --
    Intake intakeMech;
    Drivetrain drivetrain;
    Stopper stopper;
    Park park;

    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    Robot robot;

    @BeforeEach
    public void setUp() {
        shooter = new Shooter(outtake1, outtake2, hood);
        intakeMech = new Intake(intake1, intake2);
        drivetrain = new Drivetrain(leftFront, rightFront, leftBack, rightBack, imu);
        stopper = new Stopper(stopperServo);
        park = new Park(park1, park2);

        robot = new Robot(controls, imu, pinpoint, webCam, intakeMech, stopper,
                shooter, drivetrain, led0, led1, park);
    }


    @Test
    public void testShooterConversion() {
        when(webCam.getATdist()).thenReturn(48.0);
        robot.limelight = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.limelight.getATdist());

        double actualRPM = shooter.calculateShooterRPM(actualMPS);

        assertEquals(actualRPM, 622.167071180941);
    }

    @Test
    public void testShooterCalculation48() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(48.0);
        robot.limelight = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.limelight.getATdist());

        assertEquals(3.99812243100862, actualMPS);
    }

    @Test
    public void testShooterCalculation100() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(100.0);
        robot.limelight = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.limelight.getATdist());

        assertEquals(5.353239674370232, actualMPS);
    }

    @Test
    public void testHoodCalculation() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(48.0);
        robot.limelight = webCam;

        double hoodPos = shooter.calculateHoodPos(robot.limelight.getATdist());

        assertEquals(0.37663332819326245, hoodPos);
    }
}
