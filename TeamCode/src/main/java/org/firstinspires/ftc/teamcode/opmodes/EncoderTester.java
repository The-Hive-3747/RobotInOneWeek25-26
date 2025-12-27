package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
@TeleOp(name = "encoder Tester")
public class EncoderTester extends NextFTCOpMode {

    DcMotorEx flywheelBottom, flywheelTop, turret, intakeMotor;
    @Override
    public void onInit() {
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        //flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelTop = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelTop");
        //flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");

        flywheelTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    @Override
    public void onUpdate() {
        telemetry.addData("flywheelBottom", flywheelBottom.getCurrentPosition());
        telemetry.addData("flywheelTop", flywheelTop.getCurrentPosition());
        telemetry.addData("flywheelBottom vel", flywheelBottom.getVelocity());
        telemetry.addData("flywheelTop vel", flywheelTop.getVelocity());
        telemetry.addData("intakeMotor", intakeMotor.getCurrentPosition());
        telemetry.addData("turret", turret.getCurrentPosition());
        telemetry.addData("port fly bottom", ActiveOpMode.hardwareMap().get("flywheelBottom").getConnectionInfo());
        telemetry.addData("port fly top", ActiveOpMode.hardwareMap().get("flywheelTop").getConnectionInfo());
        telemetry.addData("port intake", ActiveOpMode.hardwareMap().get("transfer").getConnectionInfo());
        telemetry.addData("port turret", ActiveOpMode.hardwareMap().get("turret").getConnectionInfo());
        telemetry.update();
    }


}
