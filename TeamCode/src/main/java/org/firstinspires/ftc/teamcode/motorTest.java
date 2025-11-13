package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

import dev.nextftc.core.components.BindingsComponent;
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
    MotorEx frontLeft, frontRight, backLeft, backRight;


    @Override
    public void onInit() {
        frontLeft = new MotorEx("frontLeftMotor").brakeMode();
        frontRight = new MotorEx("frontRightMotor").brakeMode();
        backLeft = new MotorEx("backLeftMotor").brakeMode();
        backRight = new MotorEx("backRightMotor").brakeMode();
    }

    @Override
    public void onUpdate() {
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
