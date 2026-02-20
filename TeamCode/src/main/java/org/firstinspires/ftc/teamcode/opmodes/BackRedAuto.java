package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.intake1;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.intake2;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.intake3;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.lineUpForIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.lineUpForIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.lineUpForIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.park;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.toShootFromIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.toShootFromIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.toShootFromIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.AutoPaths.toShootFromStart;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(name = "back red auto")
public class BackRedAuto extends AutoTemplate {
  @Override
  public void preInit() {
    startPose = new Pose(86.5,8.62,90);
    alliance = Alliance.RED;
    autonomousCommands = new SequentialGroup(
            startIntakeFlywheelAndTurret(),
            new FollowPath(toShootFromStart),
            new Delay(DELAY_BEFORE_SHOT),
            shootAllThree(),
            followPathAndIntake(lineUpForIntake1, intake1),
            followPathAndShoot(toShootFromIntake1),
            followPathAndIntake(lineUpForIntake2, intake2),
            followPathAndShoot(toShootFromIntake2),
            followPathAndIntake(lineUpForIntake3, intake3),
            followPathAndShoot(toShootFromIntake3),
            stopIntakeFlywheelAndTurret(),
            new FollowPath(park)
      );
    }
}
