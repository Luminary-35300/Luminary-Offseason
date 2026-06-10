package org.firstinspires.ftc.teamcode.NextFTC.autonomous.Close.blue;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.BaseShooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Blue 18 Gate + HoldFirst Gate")
public class Blue18Gate_HoldFirst extends NextFTCOpMode {
    public Blue18Gate_HoldFirst() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        BaseShooternf.INSTANCE, Stoppernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public Pose scorePose = new Pose(54, 94);
    public double scoreHeading = 137;
    public PathChain shootPreloads;
    public PathChain grabMiddleSet;
    public PathChain gateSet3;
    public PathChain shootMiddleSet;
    public PathChain gateIntake;
    public PathChain shootGate;

    public PathChain grabSet2;
    public PathChain shootSet2;

    public void buildPaths() {
        follower().setStartingPose(new Pose(17.8, 118.6, Math.toRadians(144)));

        shootPreloads = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.793, 118.616),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(scoreHeading))
                .build();

        grabMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(50, 59),
                                new Pose(40, 59),
                                new Pose(10, 59)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        gateSet3 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10, 59),
                                new Pose(36.496, 70),
                                new Pose(17, 70)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shootMiddleSet = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17, 70),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(scoreHeading))

                .build();

        gateIntake = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847),
                                new Pose(11.314, 60.731)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();

        shootGate = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.314, 60.731),
                                new Pose(22.357, 48.847),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();

        grabSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(40.256, 83.257),
                                new Pose(17, 83.257)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shootSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17, 83.257),

                                new Pose(54.237, 108.727)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                .build();

    }




    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.farSide(),
                Stoppernf.INSTANCE.stop(),
                Intakenf.INSTANCE.pivotDown()
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                BaseShooternf.INSTANCE.setShooterVel(1030),
                new SequentialGroup(

                    //Score Preloads
                    new ParallelDeadlineGroup(
                            f.i.follow(shootPreloads), //if no move on, check to see if release command finishes
                            Stoppernf.INSTANCE.release()
                    ),
                    s.i.shoot(1),


                    //Middle Set
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabMiddleSet),
                                            Stoppernf.INSTANCE.stop()
                                    ),
                                    new ParallelGroup(
                                            f.i.follow(gateSet3),
                                            Stoppernf.INSTANCE.stop()

                                    ),
                                    Intakenf.INSTANCE.off(),
                                    new Delay(0.7),
                                    new ParallelGroup(
                                            f.i.follow(shootMiddleSet),
                                            Stoppernf.INSTANCE.release()
                                    )
                            ),
                            new SequentialGroup(
                                    new WaitUntil(() -> shootMiddleSet.lastPath().getDistanceRemaining() < 2),
                                    s.i.shoot(0.6)
                            )
                    ),


                    //Gate Cycles
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(gateIntake),
                                            Stoppernf.INSTANCE.stop()
                                    ),
                                    new Delay(1.4),
                                    Intakenf.INSTANCE.off(),
                                    new ParallelGroup(
                                            f.i.follow(shootGate),
                                            Stoppernf.INSTANCE.release()

                                    )
                            ),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                    s.i.shoot(0.6)
                            )
                    ),
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(gateIntake),
                                            Stoppernf.INSTANCE.stop()
                                    ),
                                    new Delay(1.4),
                                    Intakenf.INSTANCE.off(),
                                    new ParallelGroup(
                                            f.i.follow(shootGate),
                                            Stoppernf.INSTANCE.release()

                                    )
                            ),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                    s.i.shoot(0.6)
                            )
                    ),
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(gateIntake),
                                            Stoppernf.INSTANCE.stop()
                                    ),
                                    new Delay(1.4),
                                    Intakenf.INSTANCE.off(),
                                    new ParallelGroup(
                                            f.i.follow(shootGate),
                                            Stoppernf.INSTANCE.release()

                                    )
                            ),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate.lastPath().getDistanceRemaining() < 2),
                                    s.i.shoot(0.6)
                            )
                    ),


                    //Set2
                    new ParallelGroup(
                            new SequentialGroup(
                                    new ParallelGroup(
                                            f.i.follow(grabSet2),
                                            Stoppernf.INSTANCE.stop()
                                    ),
                                    Intakenf.INSTANCE.off(),
                                    new ParallelGroup(
                                            f.i.follow(shootGate),
                                            Stoppernf.INSTANCE.release()
                                    )

                            ),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootSet2.lastPath().getDistanceRemaining() < 2),
                                    s.i.shoot(0.6)
                            )
                    )
            )
        );
    }


    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        BaseShooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        BaseShooternf.INSTANCE.enable();
    }

    @Override
    public void onUpdate() {}

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}

