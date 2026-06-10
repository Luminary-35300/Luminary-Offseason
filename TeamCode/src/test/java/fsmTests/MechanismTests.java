package fsmTests;

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

import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@ExtendWith(MockitoExtension.class)
public class MechanismTests {
    // -- INTAKE --
    @Mock
    DcMotorEx intake1;
    @Mock
    DcMotorEx intake2;

    // -- SHOOTER --
    @Mock
    DcMotorEx outtake1, outtake2;
    @Mock
    Servo hood;

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

    @Mock
    logi webCam;

    // -- STOPPER --
    @Mock
    Servo stopperServo;

    // -- PARK --
    @Mock
    Servo park1;
    @Mock
    Servo park2;


    // -- ACTUAL OBJECTS --
    Intake intakeMech;
    Shooter shooterMech;
    Drivetrain drivetrain;
    Stopper stopper;
    Park park;

    // this may not work...
    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    Robot robot;

    @BeforeEach
    public void setUp() {
        intakeMech = new Intake(intake1, intake2);
        shooterMech = new Shooter(outtake1, outtake2, hood);
        drivetrain = new Drivetrain(leftFront, rightFront, leftBack, rightBack, imu);
        stopper = new Stopper(stopperServo);
        park = new Park(park1, park2);

        robot = new Robot(controls, imu, pinpoint, webCam, intakeMech, stopper,
                shooterMech, drivetrain, led0, led1, park);
    }

    // 12 TESTS

    // -- SHOOTER --
    @Test
    public void shootFromBack() {
        shooterMech.shootFromBack();

        verify(outtake1).setVelocity(anyDouble());
        verify(outtake2).setVelocity(anyDouble());
    }

    @Test
    public void shootFromFront() {
        shooterMech.shootFromFront();

        verify(outtake1).setVelocity(anyDouble());
        verify(outtake2).setVelocity(anyDouble());
    }

    @Test
    public void setVelocity() {
        shooterMech.setShooterVelocity(1000);

        verify(outtake1).setVelocity(anyDouble());
    }

    @Test
    public void setHoodAngle() {
        shooterMech.setHoodAngle(60);

        verify(hood).setPosition(anyDouble());
    }

    @Test
    public void setHoodFront() {
        shooterMech.setHoodAngle(.5);

        verify(hood).setPosition(.5);
    }

    @Test
    public void setHoodBack() {
        shooterMech.setHoodAngle(.175);

        verify(hood).setPosition(.175);
    }

    // -- INTAKE --
    @Test
    public void intakeOn() {
        intakeMech.intakeTransferOnClose();

        verify(intake1).setPower(-1);
    }

    @Test
    public void intakeOff() {
        intakeMech.intakeTransferOff();

        verify(intake1).setPower(0);
    }

    @Test
    public void intakeReverse() {
        intakeMech.intakeTransferReverse();

        verify(intake1).setPower(1);
    }

    // -- STOPPER --
    @Test
    public void stopperRelease() {
        stopper.release();

        verify(stopperServo).setPosition(anyDouble());
    }

    @Test
    public void stopperStop() {
        stopper.stop();

        verify(stopperServo).setPosition(anyDouble());
    }
}
