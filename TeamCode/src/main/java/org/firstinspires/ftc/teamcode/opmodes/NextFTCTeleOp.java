package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.*;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.helpers.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracking;
import org.firstinspires.ftc.teamcode.subsystems.TurretUsingOdo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                hood = new Hood(),
                flywheel = new Flywheel(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                odoTurret = new TurretUsingOdo()
        );
    }
    Hood hood;
    TurretUsingOdo odoTurret;

    Flywheel flywheel;
    double FLYWHEEL_VEL = 1300; // IN RPM
    double INTAKE_POWER = 0.9;
    private double HOOD_POSITION = 0.0;
    private DcMotor intakeMotor;
    private Servo flipper, light;
    private double FIRE_POWER = 0.9;
    private CRServo leftFireServo;
    private CRServo rightFireServo;
    private CRServo sideWheelServo;
    private CRServo hoodServo;
    Follower follower;
    public boolean isRed;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(OpModeTransfer.currentPose);
        follower.update();

        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        flipper = hardwareMap.get(Servo.class, "flipper");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        rightFireServo = hardwareMap.get(CRServo.class, "right_firewheel");
        sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
        isRed = OpModeTransfer.isRed;
        Button g1Back = button(() -> gamepad1.back);
        g1Back.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    isRed = !isRed;
                });
    }
    @Override
    public void onWaitForStart() {
        if (isRed) {
            light.setPosition(0.279);
        } else {
            light.setPosition(0.611);
        }

    }
    @Override
    public void onStartButtonPressed() {
        light.setPosition(0.388);
        follower.startTeleOpDrive();
        odoTurret.allowTurret();
        odoTurret.setAlliance(isRed);
        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g2A = button(() -> gamepad2.a);

        Button g2Up = button(() -> gamepad2.dpad_up);
        Button g2Down = button(() -> gamepad2.dpad_down);
        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Up = button(() -> gamepad1.dpad_up);
        Button g1Down = button(() -> gamepad1.dpad_down);

        g1Right.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    odoTurret.setTurretAngle(0);
                        })
                .whenBecomesFalse(() -> {
                    odoTurret.allowTurret();
                });
        g1Up.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    flywheel.setTargetVel(FLYWHEEL_VEL);
                    intakeMotor.setPower(INTAKE_POWER);
                    leftFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                })
                .whenBecomesFalse(() -> {
                    flywheel.setTargetVel(0);
                    intakeMotor.setPower(0);
                    leftFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                });
        g1Down.whenTrue(() -> flipper.setPosition(0.1))
                .whenFalse(() -> flipper.setPosition(0.52));
        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    flywheel.setTargetVel(FLYWHEEL_VEL);
                })
                .whenBecomesFalse(() -> {
                    flywheel.setTargetVel(0.0);
                });


        g2X.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(INTAKE_POWER);
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                });

        g2A.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    leftFireServo.setPower(FIRE_POWER);
                    //rightFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                })
                .whenBecomesFalse(() -> {
                    leftFireServo.setPower(0);
                    //rightFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                });



        g2B.whenTrue(() -> flipper.setPosition(0.1))
                .whenFalse(() -> flipper.setPosition(0.52));

        g2Up.whenTrue(() -> hoodServo.setPower(0.1))
            .whenFalse(() -> hoodServo.setPower(0));

        g2Down.whenTrue(() -> hoodServo.setPower(-0.1))
                .whenFalse(() -> hoodServo.setPower(0));
    }
    @Override
    public void onUpdate() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();
        hood.update();
        flywheel.update();
        BindingManager.update();
        odoTurret.setCurrentPose(follower.getPose());
        odoTurret.update();
        telemetry.update();
    }
}
