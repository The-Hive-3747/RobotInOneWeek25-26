package org.firstinspires.ftc.teamcode.vision.limelight;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.core.components.Component;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

//Eschew this because I was tripping, i will push onto git from my mac or do sum else -yers
public class LimelightComponent implements Component{
    private Limelight3A limelight;
    private boolean hasTarget = false;
    private double targetX = 0.0;
    private double targetY= 0.0;
    private double targetArea = 0.0;
    private int aprilTagId;

    public void init() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "Limelight");
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()) {
            hasTarget = true;
            targetX = result.getTx();
            targetY = result.getTy();
            targetArea = result.getTa();
        }
    }

}
