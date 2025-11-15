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
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracking;

import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "front RED auto")
public class FrontRedAuto extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new TurretTracking()
        );
    }
    CommandGroup autonomous;
    TurretTracking turret;
    Flywheel flywheel;
    Intake intake;

    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park;
    public static double shootAngle, parkAngle;
    TelemetryManager telemetryM;


    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //toShootFromStart =
        FrontBlueAutoPaths.generatePaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(startingPose);

        autonomous = new SequentialGroup(
                intake.startIntake,
                flywheel.startFlywheel,
                intake.startTransfer,
                turret.holdTurret,
                new FollowPath(toShootFromStart),
                flywheel.resetShotTimer,
                flywheel.shootAllThree,
                new FollowPath(lineUpForIntake1),
                new FollowPath(intake1),
                new FollowPath(toShootFromIntake1),
                flywheel.resetShotTimer,
                flywheel.shootAllThree,
                new FollowPath(lineUpForIntake2),
                new FollowPath(intake2),
                new FollowPath(toShootFromIntake2),
                flywheel.resetShotTimer,
                flywheel.shootAllThree,
                flywheel.stopFlywheel,
                intake.stopIntake,
                intake.stopTransfer,
                new FollowPath(park)
        );

    }

    @Override
    public void onUpdate() {
        autonomous.schedule();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.addData("path", PedroComponent.follower().getCurrentTValue());
        flywheel.update();
        telemetry.update();
    }



    public static class FrontBlueAutoPaths {
        public static void generatePaths(Follower follower) {
            startingPose = new Pose(4, 4, Math.toRadians(135));
            shootingPose = new Pose(39, 34);
            intake1StartPose = new Pose(34, 49);
            intake1EndPose = new Pose(6, 49);
            intake2StartPose = new Pose(29, 72);
            intake2EndPose = new Pose(-5, 72);
            parkPose = new Pose(19, 40);
            toShootCurvePose = new Pose(44,72);
            shootAngle = Math.toRadians(130);
            parkAngle = Math.toRadians(270);

            toShootFromStart = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startingPose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), shootAngle)
                    .build();

            lineUpForIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootingPose, intake1StartPose)
                    )
                    .setLinearHeadingInterpolation(shootAngle, Math.toRadians(180))
                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake1StartPose, intake1EndPose)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootFromIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake1EndPose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), shootAngle)
                    .build();

            lineUpForIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootingPose, intake2StartPose)
                    )
                    .setLinearHeadingInterpolation(shootAngle, Math.toRadians(180))
                    .build();

            intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(intake2StartPose, intake2EndPose)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            toShootFromIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(intake2EndPose, toShootCurvePose, shootingPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), shootAngle)
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
