package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.FollowPath;

public class f extends SubsystemGroup {
    public static final f i = new f();

    private f() {
        super(Intakenf.INSTANCE, Stoppernf.INSTANCE);
    }

    public final Command follow(PathChain path) {
        return new SequentialGroup(
                new FollowPath(path)
        );
    }

//    public final Command follow(PathChain path, boolean holdEnd) {
//        return new SequentialGroup(
//                new FollowPath(path, holdEnd)
//        );
//    }

    public final Command follow(PathChain path, boolean holdEnd, double maxPower) {
        return new SequentialGroup(
                new FollowPath(path, holdEnd, maxPower)
        );
    }




}