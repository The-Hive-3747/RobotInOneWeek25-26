package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Ri1WTeleOp extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private Servo pivotServo = null;
    private DcMotor shootMotor = null;
    private double shootPower = 0.7;
    boolean powerHasChanged = false;

    @Override
    public void runOpMode() throws InterruptedException {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        DcMotor shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");
        odo.setOffsets(-5.4, 0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            odo.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x/0.5;
            String pivotState = "DEFAULT";


            if (gamepad1.options) {
                odo.setHeading(0, AngleUnit.RADIANS);
            }

            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.left_trigger > 0.1 && pivotState == "DEFAULT") {
                //testServo.setPosition(0.6+(gamepad1.left_trigger*0.4));
                pivotServo.setPosition(1.0);
                pivotState = "LOAD";

            }else if(gamepad1.right_trigger > 0.1 && pivotState == "LOAD"){
                pivotServo.setPosition(0.6);
                pivotState = "DEFAULT";
            }else if(gamepad1.left_trigger > 0.1 && pivotState == "SHOOT"){
                pivotServo.setPosition(0.6);
                pivotState = "DEFAULT";

            } else if (gamepad1.right_trigger > 0.1 && pivotState == "DEFAULT") {
                pivotServo.setPosition(0.2);
                pivotState = "SHOOT";
            }

            if (gamepad1.x) {
                shootMotor.setPower(shootPower);
            }
            if (gamepad1.y) {
                shootMotor.setPower(0);
            }

            if (gamepad1.dpad_up && !powerHasChanged) {
                shootPower += 0.025;
                shootMotor.setPower(shootPower);
                powerHasChanged = true;
            }
            if (!gamepad1.dpad_up && powerHasChanged) {
                powerHasChanged = false;
            }
            if (gamepad1.dpad_down && !powerHasChanged) {
                shootPower -= 0.025;
                shootMotor.setPower(shootPower);
                powerHasChanged = true;
            }
            if (!gamepad1.dpad_down && powerHasChanged) {
                powerHasChanged = false;
            }


            telemetry.addData("shooter power", shootPower);
            telemetry.update();


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}