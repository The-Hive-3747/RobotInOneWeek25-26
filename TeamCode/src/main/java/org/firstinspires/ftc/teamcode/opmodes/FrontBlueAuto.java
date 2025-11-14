package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.pathing.CustomDrawing;
import org.firstinspires.ftc.teamcode.pathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.nio.file.Paths;

import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;


@Autonomous(name = "frontBlueAuto")
public class FrontBlueAuto extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake()
        );
    }
    CommandGroup autonomous;
    Flywheel flywheel;
    Intake intake;
    Paths paths;
    MotorEx frontLeft, frontRight, backLeft, backRight;

    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, intake2CurvePose, parkPose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park;
    public static double shootAngle, parkAngle;
    TelemetryManager telemetryM;


    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //toShootFromStart =
        FrontBlueAutoPaths.getPaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(startingPose);

        autonomous = new SequentialGroup(
                //new ParallelDeadlineGroup(
                  //      new FollowPath(toShootFromStart),
                        //intake.startIntake,
                    //    intake.startTransfer,
                      //  flywheel.startFlywheel
                //),
                intake.startIntake,
                flywheel.startFlywheel,
                intake.startTransfer,

                new FollowPath(toShootFromStart),
                flywheel.shootAllThree,

                new FollowPath(lineUpForIntake1)
                //new FollowPath(intake1),
                //new FollowPath(toShootFromIntake1),
                //new FollowPath(lineUpForIntake2),
                //new FollowPath(intake2),
                //new FollowPath(toShootFromIntake2),
                //new FollowPath(park)
        );

        //Tuning.drawOnlyCurrent();

    }

    @Override
    public void onUpdate() {
        autonomous.schedule();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        flywheel.update();
        telemetry.update();
    }



    public static class FrontBlueAutoPaths {
        public static void getPaths(Follower follower) {
            startingPose = new Pose(140, 140, Math.toRadians(45));
            shootingPose = new Pose(110, 110);
            intake1StartPose = new Pose(120, 110);
            intake1EndPose = new Pose(140, 110);
            intake2StartPose = new Pose(99.607, 90);
            intake2EndPose = new Pose(131.361, 90);
            intake2CurvePose = new Pose(91.484, 53.538);
            parkPose = new Pose(125.638, 101.169);
            shootAngle = Math.toRadians(50);
            parkAngle = Math.toRadians(270);

            toShootFromStart = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startingPose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), shootAngle)
                    .build();

            lineUpForIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootingPose, intake1StartPose)
                    )
                    .setLinearHeadingInterpolation(shootAngle, Math.toRadians(0))
                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake1StartPose, intake1EndPose)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            toShootFromIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake1EndPose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootAngle)
                    .build();

            lineUpForIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootingPose, intake2StartPose)
                    )
                    .setLinearHeadingInterpolation(shootAngle, Math.toRadians(0))
                    .build();

            intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake2StartPose, intake2EndPose)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            toShootFromIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(intake2EndPose, intake2CurvePose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootAngle)
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootingPose, parkPose)
                    )
                    .setLinearHeadingInterpolation(shootAngle, parkAngle)
                    .build();
        }


    }

}
