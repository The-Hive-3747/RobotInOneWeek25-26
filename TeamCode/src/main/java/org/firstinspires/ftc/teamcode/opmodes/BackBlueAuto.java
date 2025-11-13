package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathing.Constants;

import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class BackBlueAuto extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }
    CommandGroup autonomous;
    @Override
    public void onInit() {
        autonomous = new SequentialGroup(

        );
    }

    @Override
    public void onUpdate() {
        autonomous.schedule();
    }

}
