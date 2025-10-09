package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathing.Constants;

import java.util.function.Supplier;

@Autonomous
public class BlueDriveForwardShootAuto extends LinearOpMode {

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
                frontLeftMotor.setPower(0.4);
                backLeftMotor.setPower(0.4);
                frontRightMotor.setPower(0.4);
                backRightMotor.setPower(0.4);
            }
            if (runtime.seconds() > 2.5 && runtime.seconds() < 3.5){
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
            }
            if (runtime.seconds() > 3.5 && runtime.seconds() < 4.75) {
                frontLeftMotor.setPower(0.3);
                backLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
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
                frontLeftMotor.setPower(-0.3);
                backLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                backRightMotor.setPower(-0.3);
            }
            if(runtime.seconds() > 14.15){
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }
    }

    @Configurable
    @TeleOp
    public static class PedroTeleOp extends OpMode {
        private Follower follower;
        public static Pose startingPose;
        private boolean automatedDrive;
        private Supplier<PathChain> pathChain;
        private TelemetryManager telemetryM;
        private boolean slowMode = false;
        private double slowModeMultiplier = 0.5; //subject to change;
        Limelight3A limelight;

        @Override
        public void init(){
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();

            pathChain = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(45,98))))
                    //getPose, newPose subject to change
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                            Math.toRadians(45),0.8)) //subject to change
                    .build();

        }

        @Override
        public void start() {

        }
        @Override
        public void loop() {

        }

    }
}
