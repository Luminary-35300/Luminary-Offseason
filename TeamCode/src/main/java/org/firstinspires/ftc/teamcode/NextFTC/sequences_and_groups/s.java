package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

//AUTON SEQUENCES CLOSE
public class s extends SubsystemGroup {
    public static final s i = new s();
    private s() {
        super(
                Intakenf.INSTANCE, Stoppernf.INSTANCE,
                BaseShooternf.INSTANCE, Hoodnf.INSTANCE
        );
    }


    /**
     * Waits until pathChain is at end
     * Opens blocker
     * Transfers up for shootTime seconds
     * Close blocker
     * @param shootTime time transfer shoots for
     * @param pathChain last pathChain of sequence
     * @return Sequential Group
     */
    public Command shootSequence(PathChain pathChain, double shootTime) {
        return new SequentialGroup(
                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                shoot(shootTime)
        );
    }

    public Command shootSequence(PathChain pathChain, double shootTime, double spinUpTime) {
        return new SequentialGroup(
                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                new Delay(spinUpTime),
                shoot(shootTime)
        );
    }


    public Command shootAt(Follower follower, PathChain pathChain, double shootTime, double shootTValue) {
        return new SequentialGroup(
                new WaitUntil(() -> follower.getCurrentTValue() >= shootTValue ),
                shoot(shootTime)
        );
    }

    /**
     * Blocker opens
     * Transfers up for shootTime seconds
     * Blocker closes
     * @param shootTime
     * @return
     */
    public Command shoot(double shootTime) {
        return new ParallelGroup(
                new SequentialGroup(
                        Stoppernf.INSTANCE.release(),
                        new Delay(shootTime),
                        Stoppernf.INSTANCE.stop()
                ),
                Intakenf.INSTANCE.in(),
                Intakenf.INSTANCE.pivotDown()
        );
    }

    /**
     * Blocker opens
     * Transfers up for shootTime seconds
     * Blocker closes
     * @param shootTime
     * @return
     */
    public Command shoot(double shootTime, double transferSpeed) {
        return new ParallelGroup(

                new SequentialGroup(
                        Intakenf.INSTANCE.setIntakePower(transferSpeed),

                        Stoppernf.INSTANCE.release(),
                        new Delay(shootTime),

                        new ParallelGroup(
                                Stoppernf.INSTANCE.stop(),
                                Intakenf.INSTANCE.in()
                        )
                )
        );
    }

    /**
     * Sets shooter speed
     * Sets hood angle to default 0.3
     * @param velocity
     * @return
     */
    public Command shooterState(double velocity) {
        return new ParallelGroup(
                BaseShooternf.INSTANCE.setShooterVel(velocity),
                Hoodnf.INSTANCE.closeSide()
        );
    }

    /**
     * Sets shooter speed
     * Sets hood angle to hoodPos
     * @param velocity
     * @param hoodPos
     * @return
     */
    public Command shooterState(double velocity, double hoodPos) {
        return new ParallelGroup(
                BaseShooternf.INSTANCE.setShooterVel(velocity),
                Hoodnf.INSTANCE.setHoodPos(hoodPos)
        );
    }


}