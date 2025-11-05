package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.*;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE
        );
    }
    FieldCentricDrive drive;
    double FLYWHEEL_POWER = 0.6;
    double INTAKE_POWER = 0.9;
    private double HOOD_POSITION = 0.0;
    GoBildaPinpointDriver odo;
    Limelight3A limelight;
    private DcMotor intakeMotor;
    private Servo flipper;

    double x;
    double y;
    double z;
    double center;
    double centerP;

    double horizDistance;

    @Override
    public void onInit() {
        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g1Up = button(() -> gamepad1.dpad_up);
        Button g1Down = button(() -> gamepad1.dpad_down);
        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);
        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);

        /*Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");*/
        DcMotor flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        DcMotor turretMotor = hardwareMap.get(DcMotor.class, "turret");
        Servo hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        flipper = hardwareMap.get(Servo.class, "flipper");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();*/
        telemetry.addData("Servo Position", hoodServo.getPosition());


        /*g2Y.whenBecomesTrue(() -> flywheelMotor.setPower(FLYWHEEL_POWER))
                .whenBecomesFalse(() -> flywheelMotor.setPower(0));*/

        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    flywheelMotor.setPower(FLYWHEEL_POWER);
                })
                .whenBecomesFalse(() -> {
                    flywheelMotor.setPower(0);
                });


        g2X.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(INTAKE_POWER);
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                });

        g2B.whenTrue(() -> flipper.setPosition(0.27))
                .whenFalse(() -> flipper.setPosition(0.4));

        if (0>gamepad2.left_stick_y) {
            if (hoodServo.getPosition() <= 0.1) {
                HOOD_POSITION = 0;
                hoodServo.setPosition(HOOD_POSITION);
            } else {
                HOOD_POSITION -= 0.1;
                hoodServo.setPosition(HOOD_POSITION);
            }
        }
        if (0<gamepad2.left_stick_y) {
            if (hoodServo.getPosition() >= 0.9) {
                HOOD_POSITION = 1;
                hoodServo.setPosition(HOOD_POSITION);
            } else {
                HOOD_POSITION += 0.1;
                hoodServo.setPosition(HOOD_POSITION);
            }
        }
        //g2Y.whenBecomesTrue(() -> shootMotor.setPower(0));

        /*
        g1LT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    pivotServo.setPosition(0.2);
                })
                .whenBecomesFalse(() -> {
                    pivotServo.setPosition(0.6);
                });*/
        /*g1Left.whenTrue(() -> {
            turretMotor.setPower(0.15);
                });

        g1Right.whenTrue(() -> {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            turretMotor.setPower(0.15);
                });*/
    }
    @Override
    public void onUpdate() {
        //LLResult result = limelight.getLatestResult();
        /*if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            Pose3D botPose = fiducial.getRobotPoseTargetSpace();
            centerP = fiducial.getTargetXPixels();
            center = fiducial.getTargetXDegrees();
            y = botPose.getPosition().y;
            z = botPose.getPosition().z;
            x = botPose.getPosition().x;

            horizDistance = Math.sqrt(x*x + y*y);
            ActiveOpMode.telemetry().addData("horizontal distance", horizDistance);
            ActiveOpMode.telemetry().addData("april tag", fiducial.getFiducialId());
            ActiveOpMode.telemetry().addData("z distance", z);
            ActiveOpMode.telemetry().addData("x distance", x);
            ActiveOpMode.telemetry().addData("y distance", y);
            ActiveOpMode.telemetry().addData("center distance", center);
        }*/


        BindingManager.update();
        drive.update();
        telemetry.update();
    }
}