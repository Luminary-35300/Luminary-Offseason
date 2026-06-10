package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class BaseShooternf implements Subsystem {
    public static final BaseShooternf INSTANCE = new BaseShooternf();
    private BaseShooternf() { }

    public MotorEx outtake1, outtake2;
    public MotorGroup shooter;

    private final ControlSystem closeShooterController = ControlSystem.builder()
            .velPid(20, 0, 0.002)
            //old 3.5, 0.002, ff = 0.001
            .basicFF(0.003)
            .build();



    private boolean enabled = false;

    public Command closeSide() {
        return new RunToVelocity(closeShooterController, 1100).requires(shooter);
    }

    public Command farSide() {
        return new RunToVelocity(closeShooterController, 1450).requires(shooter);
    }

    public Command idle() {
        return new RunToVelocity(closeShooterController, 0).requires(shooter);
    }

    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(closeShooterController, shooterVel).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0);
    }

    @Override
    public void initialize() {
        outtake1 = new MotorEx("ol");
        outtake2 = new MotorEx("or");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        disable();
    }

    @Override
    public void periodic() {
        if (!enabled) {
            shooter.setPower(0);
            return;
        }

        shooter.setPower(closeShooterController.calculate(shooter.getState()));
    }
}
