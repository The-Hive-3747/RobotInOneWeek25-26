package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;


public class Aimbot implements Component{
    GoBildaPinpointDriver odo;
    public void preInit() {
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");

    }
    public void postInit() {
        odo.setOffsets(-5.4, 0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

    }
    public void update() {
        odo.update();
    }
}
