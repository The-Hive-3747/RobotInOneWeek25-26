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
        FrontRedAutoPaths.generatePaths(PedroComponent.follower());

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



    public static class FrontRedAutoPaths {
        public static void generatePaths(Follower follower) {
            startingPose = new Pose(140, 140, Math.toRadians(45));
            shootingPose = new Pose(105, 110);
            intake1StartPose = new Pose(110, 95);
            intake1EndPose = new Pose(138, 95);
            intake2StartPose = new Pose(115, 72);
            intake2EndPose = new Pose(149, 72);
            parkPose = new Pose(125.638, 104);
            toShootCurvePose = new Pose(100,72);
            shootAngle = Math.toRadians(50);
            parkAngle = Math.toRadians(90);

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
                            new BezierCurve(intake2EndPose, toShootCurvePose, shootingPose)
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
