package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp(name= "motor test")


public class motorTest extends NextFTCOpMode {
    {
        addComponents(
                drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE
        );
    }
    FieldCentricDrive drive;
    MotorEx frontLeft, frontRight, backLeft, backRight, flyLeft, flyRight, turret;
    CRServo leftFireServo, rightFireServo, sideWheelServo, hood;



    @Override
    public void onInit() {
        frontLeft = new MotorEx("frontLeftMotor").brakeMode();
        frontRight = new MotorEx("frontRightMotor").brakeMode();
        backLeft = new MotorEx("backLeftMotor").brakeMode();
        backRight = new MotorEx("backRightMotor").brakeMode();
        flyLeft = new MotorEx("flywheelLeft");
        flyRight = new MotorEx("flywheelRight");
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "left_firewheel");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "right_firewheel");
        sideWheelServo = ActiveOpMode.hardwareMap().get(CRServo.class, "side-wheel");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        rightFireServo.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void onUpdate() {
        if (gamepad1.dpad_up) {
            flyLeft.setPower(1); } else {
            flyLeft.setPower(0);
        }
        if (gamepad1.dpad_down) {
            flyRight.setPower(1); } else {
            flyRight.setPower(0);
        }
        if (gamepad1.a) {
            frontLeft.setPower(1);
        } else {
            frontLeft.setPower(0);
        }
        if (gamepad1.b) {
            frontRight.setPower(1);
        } else {
            frontRight.setPower(0);
        }
        if (gamepad1.x) {
            backLeft.setPower(1);
        } else {
            backLeft.setPower(0);
        }
        if (gamepad1.y) {
            backRight.setPower(1);
        } else {
            backRight.setPower(0);
        }
    }
}
