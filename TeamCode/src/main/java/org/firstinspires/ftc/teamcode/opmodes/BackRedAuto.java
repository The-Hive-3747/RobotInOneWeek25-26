package org.firstinspires.ftc.teamcode.opmodes;


@Autonomous(name = "back red auto")
public class BackRedAuto extends AutoTemplate {
  @Override
  public void preInit() {
    super.startingPose = new Pose(86.5,8.62,90);
    super.alliance = Alliance.RED;
    super.autonomousCommands = new SequentialGroup(
                startIntakeFlywheelAndTurret, 
                new Delay(5),
                followPathAndShoot(toShootFromStart),
                followPathAndIntake(lineUpForIntake1, intake1),
                followPathAndShoot(toShootFromIntake1),
                followPathAndIntake(lineUpForIntake2, intake2),
                followPathAndShoot(toShootFromIntake2),
                followPathAndPark(park) 
        );
  }
}
