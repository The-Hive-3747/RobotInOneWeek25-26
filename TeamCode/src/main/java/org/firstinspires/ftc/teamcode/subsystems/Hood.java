package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Hood implements Component {
    CRServo hood;
    AnalogInput hoodEncoder;
    @Override
    public void postInit() {
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        hoodEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "hoodEncoder");

    }

    public void update() {
        if (ActiveOpMode.gamepad1().dpad_up) {
            hood.setPower(1.0);
        } else if (ActiveOpMode.gamepad1().dpad_down){
            hood.setPower(-1);
        } else {
            hood.setPower(0.0);
        }
        ActiveOpMode.telemetry().addData("encoder current pos", (hoodEncoder.getVoltage()/3.3)*360);
        ActiveOpMode.telemetry().addData("encoder max pos", hoodEncoder.getMaxVoltage());
    }
}
