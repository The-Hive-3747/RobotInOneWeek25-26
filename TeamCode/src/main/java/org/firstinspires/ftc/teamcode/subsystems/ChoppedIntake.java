package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.components.Component;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.ftc.ActiveOpMode;

public class ChoppedIntake implements Component {
    private DcMotor intakeMotor;
    private ColorSensor colorSensor;

    public enum ArtifactColor {
        NONE,
        PURPLE,
        GREEN
    }
    public int alpha =0;
    public int red = 0;
    public int green =0;
    public int blue = 0;
    private ArtifactColor storedArtifact = ArtifactColor.NONE;
    private boolean artifactDetectedThisRun = false;

    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intakeMotor");
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "colorSensor");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(double power) {
        intakeMotor.setPower(power);
        colorSensor.enableLed(true);
        if (power > 0) {
            checkForArtifact();
        }
    }
    public void stop() {
        intakeMotor.setPower(0);
        colorSensor.enableLed(false);
    }
    private void checkForArtifact() {
        alpha = colorSensor.alpha();
        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        if (storedArtifact == ArtifactColor.NONE) {
            if(alpha>500) {
                if (blue >red && blue >green) {
                    storedArtifact = ArtifactColor.PURPLE;
                }
                else if (green > red && green>blue) {
                    storedArtifact = ArtifactColor.GREEN;
                }
            }
        }

    }
    public ArtifactColor getStoredArtifact() {
        return storedArtifact;
    }
    public void reset() {
        storedArtifact = ArtifactColor.NONE;
        //artifactDetectedThisRun = false;
    }
}
