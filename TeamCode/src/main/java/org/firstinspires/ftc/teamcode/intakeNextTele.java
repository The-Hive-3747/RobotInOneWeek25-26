package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name= "NextIntake Tele")


public class intakeNextTele extends NextFTCOpMode {
    {
        addComponents(
                drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE
        );
    }
    FieldCentricDrive drive;


    @Override
    public void onInit() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        CRServo leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        CRServo rightFireServo = hardwareMap.get(CRServo.class, "right_firewheel");
        DcMotor flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        CRServo sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}
