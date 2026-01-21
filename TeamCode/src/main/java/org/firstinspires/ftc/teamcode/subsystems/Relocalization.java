package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.components.Component;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.ActiveOpMode;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Relocalization implements Component{
Limelight3A limelight;

@Override
    public void preInit() {
    limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
    //limelight.start();
    limelight.shutdown();

}
    public void update() {
    LLResult result = limelight.getLatestResult();
    if (result != null && result.isValid()) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null) {
            //Convert the Pose3D to a Pose2D in order to convert to Pedro
            Pose2D botPose2D = new Pose2D(DistanceUnit.INCH ,botPose.getPosition().x, botPose.getPosition().y, AngleUnit.RADIANS ,botPose.getOrientation().getYaw());
            //Convert the Pose2D into Pedro Pose
            Pose botPosePedro = PoseConverter.pose2DToPose(botPose2D, InvertedFTCCoordinates.INSTANCE);
            //Make the coordinate system to Pedro's coordinate system.
            botPosePedro.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            double x = botPosePedro.getX();
            double y = botPosePedro.getY();
            double heading = botPosePedro.getHeading();

            ActiveOpMode.telemetry().addData("Bot X", x);
            ActiveOpMode.telemetry().addData("Bot Y", y);
            ActiveOpMode.telemetry().addData("Bot Heading", Math.toDegrees(heading));
            ActiveOpMode.telemetry().addData("Bot pos", botPose.getPosition().toUnit(DistanceUnit.INCH));

        }
    }
}



}
