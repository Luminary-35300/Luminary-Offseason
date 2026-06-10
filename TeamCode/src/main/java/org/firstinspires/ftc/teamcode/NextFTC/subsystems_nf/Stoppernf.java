package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Stoppernf implements Subsystem {
    public static final Stoppernf INSTANCE = new Stoppernf();
    private Stoppernf() {}

    public ServoImplEx stopper;

    public Command release() {
        return new InstantCommand(() -> stopper.setPosition(1));
    }

    public Command stop() {
        return new InstantCommand(() -> stopper.setPosition(0.8));
    }

    @Override
    public void initialize() {
        stopper = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "stopper");
    }

    @Override
    public void periodic() {}
}