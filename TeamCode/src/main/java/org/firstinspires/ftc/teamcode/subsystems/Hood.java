package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Hood implements Component {
    CRServo hood;
    AnalogInput hoodEncoder;
    static double hoodPos;
    static double pastPos = 0;
    static double currentPos;
    static double loops;
    static double targetPos;
    ControlSystem hoodPID;
    
    @Override
    public void postInit() {
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        hoodEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "hoodEncoder");
        hoodPos = 0;
        loops = 1;
        ControlSystem hoodPID = ControlSystem.builder()
            .posPID(0.01)
            .build();
    }

    public double getPos() {
        return this.getRawPos()+(loops*360);
    }

    private double getRawPos() {
        return hoodEncoder.getVoltage()/3.3)*360;
    }

    public void goToPos(double pos) {
        targetPos = pos;
        hoodPid.setGoal(
            new KineticState(targetPos)
            );
    }


    public void update() {
        hood.setPower(hoodPid.calculate(
            new KineticState(getPos())
        ));
        
        currentPos = getRawPos();
        if (pastPos == 0) { pastPos = currentPos; }
        if (pastPos - currentPos > 100) { loops++; }
        
        ActiveOpMode.telemetry().addData("encoder current pos", (hoodEncoder.getVoltage()/3.3)*360);
        ActiveOpMode.telemetry().addData("encoder max pos", hoodEncoder.getMaxVoltage());
    }
}
