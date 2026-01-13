
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

@Autonomous(name = "back red auto")
public class AutoTemplate extends AutoTemplate {
   super.startingPose = new Pose(72,72,90);
  super.alliance = Alliance.RED;
  super.autonomousCommands = foo;
}
