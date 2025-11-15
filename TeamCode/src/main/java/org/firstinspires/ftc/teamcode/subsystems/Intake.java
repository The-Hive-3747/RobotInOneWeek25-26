package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Intake implements Component {
    DcMotor intakeMotor;
    double INTAKE_POWER = 0.9;
    double FIRE_POWER = 0.9;
    CRServo leftFireServo, rightFireServo, sideWheelServo, hood;

    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "transfer");
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "left_firewheel");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "right_firewheel");
        sideWheelServo = ActiveOpMode.hardwareMap().get(CRServo.class, "side-wheel");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public Command startIntake = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(INTAKE_POWER);
            })
            .setIsDone(() -> true);;
    public Command startTransfer = new LambdaCommand()
            .setStart(() -> {
                leftFireServo.setPower(FIRE_POWER);
                //rightFireServo.setPower(FIRE_POWER);
                sideWheelServo.setPower(FIRE_POWER);
            })
            .setIsDone(() -> true);
    public Command stopIntake = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
            });
    public Command stopTransfer = new LambdaCommand()
            .setStart(() -> {
                leftFireServo.setPower(0);
                //rightFireServo.setPower(0);
                sideWheelServo.setPower(0);
            });


    public void update() {

    }
}
