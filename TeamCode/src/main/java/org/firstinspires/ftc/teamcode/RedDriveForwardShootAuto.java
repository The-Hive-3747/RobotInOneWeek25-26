package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedDriveForwardShootAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor");
        Servo pivotServo = hardwareMap.servo.get("pivotServo");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;
        runtime.reset();
        while (opModeIsActive()) {

            if (runtime.seconds() < 2.5 ) {
                frontLeftMotor.setPower(0.42);
                backLeftMotor.setPower(0.42);
                frontRightMotor.setPower(0.4);
                backRightMotor.setPower(0.4);
            }
            if (runtime.seconds() > 2.5 && runtime.seconds() < 3.5){
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
            }
            if (runtime.seconds() > 3.5 && runtime.seconds() < 4.3) {
                frontRightMotor.setPower(0.3);
                backRightMotor.setPower(0.3);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
            }
            if (runtime.seconds() > 4.3 && runtime.seconds() < 9.3){
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                shootMotor.setPower(0.85);
            }
            if(runtime.seconds() > 9.3 && runtime.seconds() < 10.8){
                pivotServo.setPosition(0.6);
            }
            if (runtime.seconds() > 10.8 && runtime.seconds() < 12.8){
                shootMotor.setPower(0.0);
                frontLeftMotor.setPower(0.0);
                backLeftMotor.setPower(0.0);
                frontRightMotor.setPower(0.0);
                backRightMotor.setPower(0.0);
                pivotServo.setPosition(0.3);
            }
            if (runtime.seconds() > 12.8 && runtime.seconds() < 14.15){
                frontLeftMotor.setPower(0.3);
                backLeftMotor.setPower(-0.3);
                frontRightMotor.setPower(-0.3);
                backRightMotor.setPower(0.3);
            }
            if(runtime.seconds() > 14.15){
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }
    }
}
