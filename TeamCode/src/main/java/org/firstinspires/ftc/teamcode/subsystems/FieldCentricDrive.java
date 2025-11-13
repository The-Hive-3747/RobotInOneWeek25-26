package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.core.components.Component;
import dev.nextftc.hardware.impl.MotorEx;

public class FieldCentricDrive implements Component {
    GoBildaPinpointDriver odo;
    MotorEx frontLeft, frontRight, backLeft, backRight;
    Gamepad gamepad;
    @Override
    public void preInit() {
        frontLeft = new MotorEx("frontLeftMotor").brakeMode();
        frontRight = new MotorEx("frontRightMotor").brakeMode();
        backLeft = new MotorEx("backLeftMotor").brakeMode();
        backRight = new MotorEx("backRightMotor").brakeMode();
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
    }

    @Override
    public void postInit() {
        odo.setOffsets(-5.4, 0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        gamepad = ActiveOpMode.gamepad1();
        odo.resetPosAndIMU();
        //frontLeft.reverse();
        //backLeft.reverse();
        frontRight.reverse();
        backRight.reverse();
    }

    public void update() {
                    odo.update();
                    double y = gamepad.left_stick_y ; //y
                    double x = -gamepad.left_stick_x;
                    double rx = 0.5 * -gamepad.right_stick_x; //rx

                    double botHeading = odo.getHeading(AngleUnit.RADIANS);
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;//rotY + rotX + rx
                    double backLeftPower = (rotY - rotX + rx) / denominator;//rotY - rotX + rx
                    double frontRightPower = (rotY - rotX - rx) / denominator;//rotY - rotX - rx
                    double backRightPower = (rotY + rotX - rx) / denominator;//rotY + rotX - rx

                    frontLeft.setPower(frontLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                    backLeft.setPower(backLeftPower);

                    ActiveOpMode.telemetry().addData("heading", odo.getHeading(AngleUnit.DEGREES));
                    ActiveOpMode.telemetry().addData("Position X", odo.getPosX(DistanceUnit.INCH));
                    ActiveOpMode.telemetry().addData("Position Y", odo.getPosY(DistanceUnit.INCH));

    }
    }