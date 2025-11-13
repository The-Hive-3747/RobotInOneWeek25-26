package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathing.Constants;

import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class testAuto extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }
    CommandGroup autonomous;
    @Override
    public void onInit() {
        PathChain test = PedroComponent.follower()
                .pathBuilder()
                //.setVelocityConstraint(0.4)
                .addPath(new BezierLine(new Pose(100,100), new Pose(100, 50)))
                .setConstantHeadingInterpolation(0)
                .build();
        PedroComponent.follower().setStartingPose(new Pose(100,100));
        autonomous = new SequentialGroup(
            new FollowPath(test)
        );
    }

    @Override
    public void onUpdate() {
        autonomous.schedule();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

}
