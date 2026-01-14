package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startAngle;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startingPose;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toEndFromStart;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Light;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.follower.Follower;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "masters red auto")
public class MastersAutoRed extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret(),
                light = new Light()
        );
    }
    CommandGroup autonomous;
    Light light;
    Flywheel flywheel;
    Turret turret;
    TurretLights turretLights;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;

    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        BackAutoPaths.alliance = Alliance.RED;
        BackAutoPaths.generatePaths(PedroComponent.follower());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startingPose.getX(), startingPose.getY(), startAngle));
        follower.update();

        turret.setAlliance(Alliance.RED);
        turret.setFixedAngle(Turret.AUTON_RED_SHOOT_ANGLE);


        turretLights = new TurretLights(hardwareMap, telemetry);

        if (BackAutoPaths.getAlliance() == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
            //light.setColor(Light.COLOR_BLUE);
        }

        autonomous = new SequentialGroup(
                new ParallelGroup(
                        turret.setTurretFixed,
                        new FollowPath(toEndFromStart)
                        )
        );

        turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        //flywheel.setHoodGoalPos(1247);
        //flywheel.update();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        //turret.zeroTurret();
        flywheel.resetHoodEncoder();
        autonomous.schedule();
    }
    @Override
    public void onUpdate() {
        //turret.setCurrentPose(PedroComponent.follower().getPose(), PedroComponent.follower().getVelocity());
        turret.update();
        follower.update();

        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.addData("aimbot pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void onStop() {
        OpModeTransfer.currentPose = PedroComponent.follower().getPose();
        OpModeTransfer.alliance = Alliance.RED;
        OpModeTransfer.hasBeenTransferred = true;
    }
}
