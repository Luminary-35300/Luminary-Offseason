package org.firstinspires.ftc.teamcode.NextFTC.autonomous.Close.blue;

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
@Autonomous(name = "Blue 15 + 1 Gate")
public class Blue15Ball1Gate extends NextFTCOpMode {
    public Blue15Ball1Gate() {
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
    public Pose scorePose = new Pose(53.5, 94);
    public double scoreHeading = 138;
    // Gate Parameters ---------------------------------------------------------------
    public Pose gatePose = new Pose(14, 62.731);;
    public double gateHeading = 158.5;

    // Paths ---------------------------------------------------------------
    public PathChain shootPreloads;
    public PathChain grabMiddleSet;
    public PathChain shootMiddleSet;
    public PathChain gateIntake1;
    public PathChain gateMove1;
    public PathChain shootGate1;
    public PathChain gateIntake2;
    public PathChain gateMove2;
    public PathChain shootGate2;
    public PathChain grabSet2;
    public PathChain gateSet2;
    public PathChain shootSet2;
    public PathChain grabSet4;
    public PathChain gateSet4;
    public PathChain shootSet4;

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
                                new Pose(11, 58.7)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.55,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(scoreHeading),
                                                Math.toRadians(180)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(180))
                                )
                        )
                )
                .build();

        shootMiddleSet = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11, 58.7),
                                new Pose(48.149, 57.551),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(scoreHeading))

                .build();

        gateIntake1 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847),
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
        gateMove1 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                gatePose,
                                new Pose(11, 53.967)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(142))

                .build();
        shootGate1 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11, 53.967),
                                new Pose(22.357, 48.847),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();

        gateIntake2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(22.357, 48.847),
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
                                gatePose,

                                new Pose(11, 53.967)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(142))

                .build();
        shootGate2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11, 53.967),
                                new Pose(22.357, 48.847),
                                new Pose(53.5, 91.7)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(scoreHeading))

                .build();
        grabSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(53.5, 91.7),
                                new Pose(40.256, 83.257),
                                new Pose(17, 83.257)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        gateSet2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.000, 83.257).mirror(),
                                new Pose(105.395, 69.000).mirror(),
                                new Pose(124.500, 69.000).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.500, 69).mirror(),
                                new Pose(53.5,100)

                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                .build();
//        grabSet4 = follower().pathBuilder().addPath(
//                        new BezierCurve(
//                                scorePose,
//                                new Pose(91.891, 36.120).mirror(),
//                                new Pose(90.794, 32.034).mirror(),
//                                new Pose(129.491, 35.240).mirror()
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180-38),Math.toRadians(180))
//                .build();
//        shootSet4 = follower().pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(129.491, 35.240).mirror(),
//                                new Pose(84.889, 108.569).mirror()
//                        )
//                ).setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        0.7,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.7,
//                                        1.0,
//                                        HeadingInterpolator.constant(Math.toRadians(gateHeading))
//                                )
//                        )
//                )
//
//                .build();
    }




    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.farSide(),
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
                new ParallelGroup(
                        new FollowPath(shootMiddleSet),

                        new SequentialGroup(
                                new WaitUntil(() -> shootMiddleSet.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
                        )
                ),

                //Gate Cycles
                new FollowPath(gateIntake1),
                new FollowPath(gateMove1),
                new Delay(0.6),
                new ParallelGroup(
                        new FollowPath(shootGate1),
                        Intakenf.INSTANCE.pivotUp(),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate1.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
                        )
                ),
                //Gate Cycles
                new FollowPath(gateIntake2),
                new FollowPath(gateMove2),
                new Delay(0.6),
                new ParallelGroup(
                        new FollowPath(shootGate2),
                        Intakenf.INSTANCE.pivotUp(),

                        new SequentialGroup(
                                new WaitUntil(() -> shootGate2.lastPath().getDistanceRemaining() < 0.2),
                                new Delay(0.2),
                                s.i.shoot(0.6)
                        )
                ),


                new FollowPath(grabSet2),
                new FollowPath(gateSet2),
                new Delay(0.6),
                new FollowPath(shootSet2),
                Stoppernf.INSTANCE.release()
//                new FollowPath(grabSet4),
//
//                new ParallelGroup(
//                        new FollowPath(shootSet4),
//
//                        new SequentialGroup(
//                                new WaitUntil(() -> shootSet4.lastPath().getDistanceRemaining() < 0.2),
//                                new Delay(0.5),
//                                s.i.shoot(0.6)
//                        )
//                )


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

