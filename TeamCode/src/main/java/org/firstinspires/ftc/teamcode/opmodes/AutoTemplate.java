package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.park;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startAngle;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startingPose;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromStart;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Light;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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

public class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                aimbot = new Aimbot(),
                turret = new Turret(),
                light = new Light()
        );
    }
    public protected CommandGroup autonomousCommands;
    public protected Alliance alliance = Alliance.BLUE;
    public protected Pose startPose = new Pose(72, 72, 90);
    Light light;
    Aimbot aimbot;
    Turret turret;
    Flywheel flywheel;
    TurretLights turretLights;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;
    double FLYWHEEL_VEL;
    double HOOD_POS;
    double DELAY_BEFORE_SHOT = 0.3, DELAY_AFTER_INTAKE = 1; 
    boolean FLYWHEEL_ON = false;

    


    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        AutoPaths.alliance = alliance;
        AutoPaths.setStartPose(startPose);
        AutoPaths.generatePaths(follower);

        follower.setStartingPose(startPose);
        follower.update();

        turret.setAlliance(alliance);
        aimbot.setAlliance(alliance);
        turret.setFixedAngle(Turret.AUTON_RED_SHOOT_ANGLE);

        turretLights = new TurretLights(hardwareMap, telemetry);

        if (alliance == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
        }
        
        if (autonomousCommands == null) {
        autonomousCommands = new SequentialGroup(
                startIntakeFlywheelAndTurret, 
                followPathAndShoot(toShootFromStart),
                followPathAndIntake(lineUpForIntake1, intake1),
                followPathAndShoot(toShootFromIntake1),
                followPathAndIntake(lineUpForIntake2, intake2),
                followPathAndShoot(toShootFromIntake2),
                followPathAndIntake(lineUpForIntake3, intake3),
                followPathAndShoot(toShootFromIntake3),
                followPathAndPark(park) 
        );
    }

        turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        flywheel.resetHoodEncoder();
        autonomousCommands.schedule();
    }
    @Override
    public void onUpdate() {
        turret.update();
        follower.update();

        if (FLYWHEEL_ON) {
            FLYWHEEL_VEL = Flywheel.AUTON_SHOOT_VEL; 
    } else {
            FLYWHEEL_VEL = 0;
        }
        HOOD_POS = Hood.AUTON_HOOD_POS; 
        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(FLYWHEEL_VEL);

        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.addData("aimbot pose", follower.getPose());
        flywheel.update();
        telemetry.update();
    }

    @Override
    public void onStop() {
        OpModeTransfer.currentPose = PedroComponent.follower().getPose();
        OpModeTransfer.alliance = Alliance.RED;
        OpModeTransfer.hasBeenTransferred = true;
    }
    public Command startAimbotFlywheel = new InstantCommand(
            () -> FLYWHEEL_ON = true
    );
    public Command setFlywheelVelFinal = new InstantCommand(
            () -> FLYWHEEL_VEL = Flywheel.AUTON_SHOOT_VEL_LAST
    );
    
    public CommandGroup followPathAndShoot(PathChain pathToFollow) {
      return new SequentialGroup(
        new FollowPath(pathToFollow),
        new Delay(DELAY_BEFORE_SHOT),
        new ParallelGroup(
              flywheel.resetShotTimer,
              flywheel.shootAllThree,
              intake.startTransfer,
              intake.slowIntake
        )
    );
  }


    public CommandGroup followPathAndIntake(PathChain lineUpPath, PathChain intakePath) {
      return new SequentialGroup(
        new ParallelGroup(
                        new FollowPath(lineUpPath),
                        intake.stopTransfer,
                        intake.fastIntake
                ),
                new FollowPath(intakePath),
                new Delay(DELAY_AFTER_INTAKE)
    );
  }

    public CommandGroup followPathAndPark(PathChain parkPath) {
      return new ParallelGroup(
                        new InstantCommand(
                                () -> flywheel.setHoodGoalPos(0)
                        ),
                        turret.setTurretForward,
                        flywheel.stopFlywheel,
                        intake.stopIntake,
                        intake.stopTransfer,
                        new FollowPath(parkPath)
                );
    }

    public CommandGroup startIntakeFlywheelAndTurret = new ParallelGroup(
                        intake.fastIntake,
                        this.startAimbotFlywheel,
                        turret.setTurretFixed
                );

  }

}
