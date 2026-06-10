package fsmTests;

import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.FSM;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.teleop.gamepad.Toggle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
public class fsmTests {

    // 12 TESTS IN THIS CLASS

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

    // -- TOGGLES --

    @Mock
    Toggle shootBack, outtake, shootFront, intakeControl, pidShoot;

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
    FSM fsm;
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
        fsm = new FSM(null, controls, robot);
    }

    // This test is broken and stupid and dumb excuse my language
    @Test
    public void testBaseToOuttake() {
        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // inject mock
        when(outtake.locked()).thenReturn(true);
        controls.outtake = outtake;

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.OUTTAKING, fsm.getState());
    }

    @Test
    public void testOuttakeToBase() {
        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.OUTTAKING);

        // inject mock
        when(outtake.locked()).thenReturn(false);
        controls.outtake = outtake;

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.BASE_STATE, fsm.getState());
    }

    @Test
    public void testBaseToHardcodedBack() {
        // inject mocks
        when(shootBack.locked()).thenReturn(true);
        controls.shootBack = shootBack;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.SHOOT_BACK, fsm.getState());
    }

    @Test
    public void testHardcodedBackToBase() {
        // inject mocks
        when(shootBack.locked()).thenReturn(false);
        controls.shootBack = shootBack;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.SHOOT_BACK);

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.BASE_STATE, fsm.getState());
    }

    @Test
    public void testBaseToHardcodedFront() {
        // inject mocks
        when(shootFront.locked()).thenReturn(true);
        controls.shootFront = shootFront;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.SHOOT_FRONT, fsm.getState());
    }

    @Test
    public void testHardcodedFrontToBase() {
        // inject mocks
        when(shootFront.locked()).thenReturn(false);
        controls.shootFront = shootFront;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.SHOOT_FRONT);

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.BASE_STATE, fsm.getState());
    }

    @Test
    public void baseIntakeOn() {
        // inject mocks
        when(intakeControl.locked()).thenReturn(true);
        controls.intakeTransfer = intakeControl;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        verify(intake1).setPower(anyDouble());
    }

    @Test
    public void baseIntakeOff() {
        // inject mocks
        when(intakeControl.locked()).thenReturn(false);
        controls.intakeTransfer = intakeControl;

        // set up states
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        verify(intake1).setPower(0);
    }

    @Test
    public void testPIDShootToBase() {
        // inject mocks
        when(pidShoot.locked()).thenReturn(false);
        controls.transfer = pidShoot;

        // set up states
        fsm.setState(FSM.FSMStates.PID_SHOOT);

        // update fsm
        fsm.update();
        assertEquals(FSM.FSMStates.BASE_STATE, fsm.getState());
    }

    @Test
    public void baseLEDATDetected() {
        // inject mocks
        when(webCam.getATdist()).thenReturn(1.0);
        robot.limelight = webCam;

        // set up states
        fsm.setControlType(FSM.ControlType.PID_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        verify(led0).setState(true);
        verify(led1).setState(true);
    }

    @Test
    public void baseLEDATUndetected() {
        // inject mocks
        when(webCam.getATdist()).thenReturn(0.0);
        robot.limelight = webCam;

        // set up states
        fsm.setControlType(FSM.ControlType.PID_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);

        // update fsm
        fsm.update();
        verify(led0).setState(true);
        verify(led1).setState(true);
    }
}