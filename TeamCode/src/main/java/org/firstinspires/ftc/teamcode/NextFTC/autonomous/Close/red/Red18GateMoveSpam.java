package org.firstinspires.ftc.teamcode.NextFTC.autonomous.Close.red;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Move Spam Red 18 Gate")
public class Red18GateMoveSpam extends NextFTCOpMode {
    public Red18GateMoveSpam() {
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


    // Score Parameters ---------------------------------------------------------------
    public Pose scorePose = new Pose(53.5, 94).mirror();
    public double scoreHeading = 180-138;
    // Gate Parameters ---------------------------------------------------------------
    public Pose gatePose = new Pose(14, 62.731).mirror();
    public double gateHeading = 180-158.5;

    public PathChain shootPreloads;
    public PathChain grabMiddleSet;
    public PathChain shootMiddleSet;
    public PathChain gateIntake1;
    public PathChain gateMove1;
    public PathChain shootGate1;
    public PathChain gateIntake2;
    public PathChain gateMove2;
    public PathChain shootGate2;
    public PathChain gateIntake3;
    public PathChain gateMove3;
    public PathChain shootGate3;
    public PathChain gateIntake4;
    public PathChain gateMove4;
    public PathChain shootGate4;
    public PathChain grabSet2;
    public PathChain shootSet2;

    public void buildPaths() {
        follower().setStartingPose(new Pose(17.8, 118.6, Math.toRadians(144)).mirror());

        shootPreloads = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.793, 118.616).mirror(),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-144), Math.toRadians(scoreHeading))
                .build();

        grabMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(50, 59).mirror(),
                                new Pose(40, 59).mirror(),
                                new Pose(10, 59).mirror()
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.55,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(180-180)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(180-180))
                                )
                        )
                )
                .build();

        shootMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10, 59).mirror(),
                                new Pose(46.862, 59).mirror(), //Tune this point for least time, optimize
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(scoreHeading))

                .build();

        gateIntake1 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847).mirror(),
                                //Control point - may have to make it go around, kind of infront, and then push to gate
                                gatePose
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.65,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(gateHeading)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.65,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
                                )
                        )
                )

                .build();
        gateMove1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.700, 62.731).mirror(),

                                new Pose(8.000, 53.967).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-155), Math.toRadians(180-142))

                .build();
        shootGate1 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 53.967).mirror(),
                                new Pose(22.357, 48.847).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();
        gateIntake2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847).mirror(),
                                //Control point - may have to make it go around, kind of infront, and then push to gate
                                gatePose
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.65,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(gateHeading)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.65,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
                                )
                        )
                )

                .build();
        gateMove2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.700, 62.731).mirror(),

                                new Pose(8.000, 53.967).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-155), Math.toRadians(180-142))

                .build();
        shootGate2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 53.967).mirror(),
                                new Pose(22.357, 48.847).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();
        gateIntake3 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847).mirror(),
                                //Control point - may have to make it go around, kind of infront, and then push to gate
                                gatePose
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.65,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(gateHeading)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.65,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
                                )
                        )
                )

                .build();
        gateMove3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.700, 62.731).mirror(),

                                new Pose(8.000, 53.967).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-155), Math.toRadians(180-142))

                .build();
        shootGate3 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 53.967).mirror(),
                                new Pose(22.357, 48.847).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();
        gateIntake4 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847).mirror(),
                                //Control point - may have to make it go around, kind of infront, and then push to gate
                                gatePose
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.65,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(gateHeading)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.65,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
                                )
                        )
                )

                .build();
        gateMove4 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.700, 62.731).mirror(),

                                new Pose(12.500, 53.967).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-155), Math.toRadians(180-142))

                .build();
        shootGate4 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.314, 60.731).mirror(),
                                new Pose(22.357, 48.847).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();
        grabSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(40.256, 83.257).mirror(),
                                new Pose(17, 83.257).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))

                .build();

        shootSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17, 83.257).mirror(),

                                new Pose(54.237, 108.727).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-144))

                .build();

    }




    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.closeSide(),
                Stoppernf.INSTANCE.stop(),
                Intakenf.INSTANCE.pivotDown()
        );
    }

    private Command autonomous() {
        //TODO - after gate intake bring pivot up, see the 15 ball 1 gate file
            return new SequentialGroup(
                    new ParallelDeadlineGroup(
                            new FollowPath(shootPreloads),
                            BaseShooternf.INSTANCE.setShooterVel(1400),

                            Intakenf.INSTANCE.in(),
                            Intakenf.INSTANCE.pivotDown()
                    ),
                    new Delay(0.5),
                    s.i.shoot(0.7),

                    //Middle
                    new FollowPath(grabMiddleSet),
                    new ParallelGroup(
                            new FollowPath(shootMiddleSet),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootMiddleSet.lastPath().getDistanceRemaining() < 0.2),
                                    new Delay(0.4),
                                    s.i.shoot(0.6)
                            )
                    ),

                    //Gate Cycles
                    //Middle
                    new FollowPath(gateIntake1),
                    new Delay(0.06),
                    new FollowPath(gateMove1),
                    new Delay(0.6),
                    new ParallelGroup(
                            new FollowPath(shootGate1),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate1.lastPath().getDistanceRemaining() < 0.2),
                                    new Delay(0.4),
                                    s.i.shoot(0.6)
                            )
                    ),

                    new FollowPath(gateIntake2),
                    new Delay(0.06),
                    new FollowPath(gateMove2),
                    new Delay(0.6),
                    new ParallelGroup(
                            new FollowPath(shootGate2),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate2.lastPath().getDistanceRemaining() < 0.2),
                                    new Delay(0.4),
                                    s.i.shoot(0.6)
                            )
                    ),

                    new FollowPath(gateIntake3),
                    new Delay(0.06),
                    new FollowPath(gateMove3),
                    new Delay(0.6),
                    new ParallelGroup(
                            new FollowPath(shootGate3),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootGate3.lastPath().getDistanceRemaining() < 0.2),
                                    new Delay(0.4),
                                    s.i.shoot(0.6)
                            )
                    ),

                    new FollowPath(grabSet2),
                    new ParallelGroup(
                            new FollowPath(shootSet2),

                            new SequentialGroup(
                                    new WaitUntil(() -> shootSet2.lastPath().getDistanceRemaining() < 0.2),
                                    new Delay(0.5),
                                    s.i.shoot(0.6)
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

