package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


public class FSM {
    // --------------- Robot & States ---------------
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.PID_CONTROL;
    private final GamepadMapping gamepad;

    // --------------- SUBSYSTEMS ---------------
    private final Intake intake;
//    private final Stopper stopper;
    private final Shooter shooter;


    // --------------- MISC ---------------
    public double lastVelo = 800;
    private ControlType savedType;
    private double FAR_THRESHOLD = 110; // TODO tune threshold

    // furthest forward for far shots
    private double MAX_HOOD_POS = 0.1;
    // furthest down for close shots
    private double MIN_HOOD_POS = 0.6;
    private double HOOD_OFFSET = 0.1; // TODO tune offset

    public FSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
//        stopper = robot.stopper;
        shooter = robot.shooter;

        savedType = ControlType.PID_CONTROL;
    }

    public void update() {
        gamepad.update();

        switch (state) {
            case BASE_STATE:

                // Intake toggle
                if (gamepad.intakeTransfer.value()) {
                    intake.intakeTransferOnClose();
                    intake.pivotDown();
                } else if (!gamepad.transfer.locked()) {
                    intake.intakeTransferOff();
                    intake.pivotUp();
                }

                // Stopper hold
                if (gamepad.transfer.locked() && type == ControlType.PID_CONTROL) {
                    intake.intakeTransferOnClose();
//                    stopper.release();
                    intake.pivotDown();
                } else {
//                    stopper.stop();
                }

                // Outtake hold
                if (gamepad.outtake.locked()) {
                    state = FSMStates.OUTTAKING;
                }

                // --------------- PID Only ---------------

                if (type == ControlType.PID_CONTROL) {



                    // variables
                    double distance = Robot.limelight.getTargetArtifactTravelDistanceX();
                    double targetVelocity = robot.shooter.calculateShooterRPM(distance) + 50;
                    double targetHoodPos;

                    // calculate target
                    if (Robot.limelight.getDistanceInches() > 100) {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance) - 0.2;
                        targetVelocity = targetVelocity + 50;
                    } else {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance);
                    }

                    if (Robot.limelight.getDistanceInches() != 0) {
                        lastVelo = targetVelocity;
                    }

                    // This should prevent the shooter from changing hood pos if it can't see the AprilTag (so if it cuts out it's fine)
                    if (Robot.limelight.getTargetArtifactTravelDistanceX() == 22) {
                        robot.shooter.setHoodAngle(shooter.variableHood.getPosition());
                        robot.shooter.setShooterVelocity(lastVelo);
                    } else {
                        // get position will get last passed position so uh hopefully that should work
                        robot.shooter.setHoodAngle(targetHoodPos);
                        robot.shooter.setShooterVelocity(targetVelocity); // TODO maybe add offset (was 50)
                    }
                }

                // --------------- Hardcoded Only ---------------
                if (gamepad.switchMode.value()) {
                    // if saved is PID (in PID mode), switch to Hardcoded
                    if (savedType == ControlType.PID_CONTROL) {
                        type = ControlType.HARDCODED_CONTROL;
                        savedType = ControlType.HARDCODED_CONTROL;
                        gamepad.switchMode.set(false);
                    // if saved is Hardcoded (in Hardcoded mode), switch to PID
                    } else if (savedType == ControlType.HARDCODED_CONTROL) {
                        type = ControlType.PID_CONTROL;
                        savedType = ControlType.PID_CONTROL;
                        gamepad.switchMode.set(false);
                    }
                }

                if (type == ControlType.HARDCODED_CONTROL) {
                    shooter.shootFromFront();
                    shooter.hoodToFront();
                }

                if (gamepad.shootBack.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_BACK;
                }

                if (gamepad.shootFront.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_FRONT;
                }

                break;

            case OUTTAKING:
                intake.intakeTransferReverse();

                if (!gamepad.outtake.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }
                break;


            // --------------- Hardcoded Only ---------------

            case SHOOT_BACK:

                shooter.shootFromBack();
                shooter.hoodToBack();
                intake.intakeTransferOnClose();

//                stopper.stop();

                if (!gamepad.shootBack.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }

                break;

            case SHOOT_FRONT:

                intake.intakeTransferOnFar();
                shooter.shootFromFront();
                shooter.hoodToFront();

//                stopper.stop();

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }

                break;
        }
    }

    public void setState(FSMStates newState) {
        state = newState;
    }

    public FSMStates getState() {
        return state;
    }

    public void setControlType(ControlType newCType) {
        type = newCType;
    }

    public ControlType getControlType() {
        return type;
    }

    public enum FSMStates {
        BASE_STATE,
        SHOOT_FRONT,
        SHOOT_BACK,
        OUTTAKING
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

    }

