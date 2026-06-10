package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Hoodnf implements Subsystem {

    public static final Hoodnf INSTANCE = new Hoodnf();

    private Hoodnf() {}

    private final ServoEx variableHood = new ServoEx("variableHood",-0.1);

    //0.96 Far - Most angle
    //0.36 Close - Least angle, most arc
    public Command closeSide() {
        return new InstantCommand(() -> variableHood.getServo().setPosition(0.25)).requires(this);
    }

    public Command farSide() {
        return new InstantCommand(() -> variableHood.getServo().setPosition(0.64)).requires(this);
    }

    public Command setHoodPos(double hoodPosition) {
        return new SetPosition(variableHood, hoodPosition).requires(this);
    }

}