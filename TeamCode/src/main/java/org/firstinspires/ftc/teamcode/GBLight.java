package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class GBLight implements Component {
    Servo light = null;
    public static double COLOR_WHITE = 1.0;

    @Override
    public void postInit() {
        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
        light.setPosition(COLOR_WHITE);
    }
}
