package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
    private ArtifactColor storedArtifact = ArtifactColor.NONE;
    private boolean artifactDetectedThisRun = false;

    public void init() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intakeMotor");
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, "colorSensor");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(double power) {
        intakeMotor.setPower(power);
        if (power > 0) {
            checkForArtifact();
        }
    }
    public void stop() {
        intakeMotor.setPower(0);
        artifactDetectedThisRun = false;
    }
    private void checkForArtifact() {
        if (artifactDetectedThisRun) {
            return;
        }
        if (colorSensor.alpha() > 200) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            if (blue > red && blue > green) {
                storedArtifact = ArtifactColor.PURPLE;
                artifactDetectedThisRun = true;
            } else if (green > red && green > blue) {
                storedArtifact = ArtifactColor.GREEN;
                artifactDetectedThisRun = true;
            }
        }
    }
    public ArtifactColor getStoredArtifact() {
        return storedArtifact;
    }
    public void reset() {
        storedArtifact = ArtifactColor.NONE;
        artifactDetectedThisRun = false;
    }
}
