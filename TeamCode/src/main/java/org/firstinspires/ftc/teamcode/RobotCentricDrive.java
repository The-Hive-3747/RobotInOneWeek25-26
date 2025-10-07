package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class RobotCentricDrive implements Component {
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

    public void update(double correctR, double correctY, double correctX) {
                    odo.update();
                    double y = -correctY; //y
                    double x = correctX;
                    double rx = 0.5 * correctR; //rx

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontLeft.setPower(frontLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                    backLeft.setPower(backLeftPower);

                    ActiveOpMode.telemetry().addData("heading", odo.getHeading(AngleUnit.DEGREES));
                }
    }