package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Relocalization implements Component{
Limelight3A limelight;

@Override
    public void preInit() {
    limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
    limelight.start();

}
    public void update() {
    LLResult result = limelight.getLatestResult();
    if (result != null && result.isValid()) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null) {
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;
            ActiveOpMode.telemetry().addData("Bot X", x);
            ActiveOpMode.telemetry().addData("Bot Y", y);

        }
    }

}



}
