package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.*;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.*;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class NextFTCTeleOp extends NextFTCOpMode {
    {
        addComponents(
                drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE
        );
    }
    FieldCentricDrive drive;
    double SHOOT_POWER = 0.6;
    GoBildaPinpointDriver odo;
    Limelight3A limelight;
    double x;
    double y;
    double z;
    double horizDistance;

    @Override
    public void onInit() {
        Button g1X = button(() -> gamepad1.x);
        Button g1Y = button(() -> gamepad1.y);
        Button g1Up = button(() -> gamepad1.dpad_up);
        Button g1Down = button(() -> gamepad1.dpad_down);
        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);

        /*Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        DcMotor shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);*/
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        g1X.whenBecomesTrue(() -> shootMotor.setPower(SHOOT_POWER));
        g1Y.whenBecomesTrue(() -> shootMotor.setPower(0));
        g1Up.whenBecomesTrue(() -> {
            SHOOT_POWER += 0.025;
            shootMotor.setPower(SHOOT_POWER);
        });
        g1Down.whenBecomesTrue(() -> {
            SHOOT_POWER -= 0.025;
            shootMotor.setPower(SHOOT_POWER);
        });
        g1LT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    pivotServo.setPosition(0.2);
                })
                .whenBecomesFalse(() -> {
                    pivotServo.setPosition(0.6);
                });*/

    }
    @Override
    public void onUpdate() {
        /*LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            Pose3D botPose = fiducial.getRobotPoseTargetSpace();
            y = botPose.getPosition().y;
            z = botPose.getPosition().z;
            x = botPose.getPosition().x;
            horizDistance = Math.sqrt(x*x + y*y);
            //ActiveOpMode.telemetry().addData("horizontal distance", horizDistance);
            ActiveOpMode.telemetry().addData("april tag", fiducial.getFiducialId());
            ActiveOpMode.telemetry().addData("z distance", z);
            ActiveOpMode.telemetry().addData("x distance", x);
            ActiveOpMode.telemetry().addData("y distance", y);

        }*/


        BindingManager.update();
        drive.update();
        telemetry.update();
    }
}