package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }


    public MotorEx transfer;
    public MotorEx dropDownIntake;
    MotorGroup intake;
    public ServoEx pivotServo;


//-------------------------------------------------------------------------------


    public Command in() {
        return new SetPower(intake, 0.9);
    }
    public Command in(double seconds) {
        return new SetPower(intake, 0.9).asDeadline(new Delay(seconds));
    }

    public Command off() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }

    public Command setIntakePower(double power) {
        return new SetPower(intake, power);
    }

    public Command pivotUp() {
        return new InstantCommand(() -> pivotServo.getServo().setPosition(0.28));
    }

    public Command pivotDown() {
        return new InstantCommand(() -> pivotServo.getServo().setPosition(0.9));
    }


//-------------------------------------------------------------------------------

    @Override
    public void initialize() {
        dropDownIntake = new MotorEx("dropdownIntake");
        transfer = new MotorEx("transfer");
        transfer.reverse();
        intake = new MotorGroup(transfer, dropDownIntake);

        pivotServo = new ServoEx("pivotServo");
    }

    @Override
    public void periodic() {}

}