package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.core.components.Component;

public class FieldCentricDrive implements Component {
    double offset = 0;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    Gamepad gamepad;
    @Override
    public void preInit() {
//        frontLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontLeftMotor");
//        frontRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontRightMotor");
//        backLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "backLeftMotor");
//        backRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "backRightMotor");
//        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "odo");
//        limelightComponent = new LimelightComponent();
//        limelightComponent.init();
    }

    @Override
    public void postInit() {
        frontLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontLeftMotor");
        frontRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontRightMotor");
        backLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "backLeftMotor");
        backRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "backRightMotor");
//        odo.setOffsets(-5.4, 0, DistanceUnit.INCH);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        gamepad = ActiveOpMode.gamepad1();
//        odo.resetPosAndIMU();
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(double heading) {
                    //limelightComponent.update();
//                    if(limelightComponent.hasTarget()) {
//                        double limelightX = limelightComponent.getTargetX();
//                        double limelightY = limelightComponent.getTargetY();
//                        double limelightHeading = limelightComponent.getTargetHeading();
//                    }


                    double y = -gamepad.left_stick_y ; //y
                    double x = gamepad.left_stick_x;
                    double rx = gamepad.right_stick_x; //rx

                    double botHeading = this.getHeading(heading);
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

    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getHeading(double heading) {
        return heading - this.offset;
    }
}