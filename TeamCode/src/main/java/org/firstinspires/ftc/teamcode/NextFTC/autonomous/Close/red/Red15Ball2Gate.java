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
@Autonomous(name = "Red 15 + 2 Gate")
public class Red15Ball2Gate extends NextFTCOpMode {
    public Red15Ball2Gate() {
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
    public double scoreHeading = 180-137;
    // Gate Parameters ---------------------------------------------------------------
    public Pose gatePose = new Pose(12.7, 62.731).mirror();;
    public double gateHeading = 180-155;

    public PathChain shootPreloads;
    public PathChain grabMiddleSet;
    public PathChain gateMiddleSet;
    public PathChain shootMiddleSet;
    public PathChain gateIntake1;
    public PathChain gateMove1;
    public PathChain shootGate1;
    public PathChain grabSet2;
    public PathChain gateSet2;
    public PathChain shootSet2;
    public PathChain grabSet4;
    public PathChain gateSet4;
    public PathChain shootSet4;

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
        gateMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 59.000),
                                new Pose(105.395, 59.000),
                                new Pose(124.500, 69.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.500, 69),
                                new Pose(85.657, 60.602), //Tune this point for least time, optimize
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
        grabSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(40.256, 83.257).mirror(),
                                new Pose(17, 83.257).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-180))

                .build();
        gateSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.000, 83.257),
                                new Pose(105.395, 69.000),
                                new Pose(124.500, 69.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        shootSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.500, 69.000),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-142))

                .build();
        grabSet4 = follower().pathBuilder().addPath(
                new BezierCurve(
                        scorePose,
                        new Pose(91.891, 36.120),
                        new Pose(90.794, 32.034),
                        new Pose(129.491, 35.240)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(38),Math.toRadians(0))
                .build();
        shootSet4 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.491, 35.240),
                                new Pose(84.889, 108.569)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
                                )
                        )
                )

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
                new FollowPath(gateMiddleSet),
                new Delay(0.6),
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
                new Delay(0.2),
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

                new FollowPath(grabSet2),
                new FollowPath(gateSet2),
                new Delay(0.6),
                new ParallelGroup(
                        new FollowPath(shootSet2),

                        new SequentialGroup(
                                new WaitUntil(() -> shootSet2.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.5),
                                s.i.shoot(0.6)
                        )
                ),
                new FollowPath(grabSet4),

                new ParallelGroup(
                        new FollowPath(shootSet4),

                        new SequentialGroup(
                                new WaitUntil(() -> shootSet4.lastPath().getDistanceRemaining() < 0.2),
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

