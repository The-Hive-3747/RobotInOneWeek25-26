package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
import dev.nextftc.ftc.*;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracking;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                hood = new Hood(),
                drive = new FieldCentricDrive(),
                flywheel = new Flywheel(),
                tracking = new TurretTracking(),
                BindingsComponent.INSTANCE
        );
    }
    Hood hood;
    FieldCentricDrive drive;
    Flywheel flywheel;
    TurretTracking tracking;
    double FLYWHEEL_POWER = 0.8;//0.7;//0.6
    double FLYWHEEL_VEL = 2000; // IN RPM
    double INTAKE_POWER = 0.9;
    private double HOOD_POSITION = 0.0;
    GoBildaPinpointDriver odo;
    private DcMotor intakeMotor;
    private Servo flipper;
    private double FIRE_POWER = 0.9;//0.3;
    private CRServo leftFireServo;
    private CRServo rightFireServo;
    private CRServo sideWheelServo;
    private Servo hoodServo;
    Follower follower;



    @Override
    public void onInit() {
        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g2A = button(() -> gamepad2.a);
        Button g2Up = button(() -> gamepad2.dpad_up);
        Button g2Down = button(() -> gamepad2.dpad_down);
        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);

        DcMotor flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        DcMotor turretMotor = hardwareMap.get(DcMotor.class, "turret");
        //hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        flipper = hardwareMap.get(Servo.class, "flipper");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        rightFireServo = hardwareMap.get(CRServo.class, "right_firewheel");
        sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        rightFireServo.setDirection(CRServo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

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
                    rightFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                })
                .whenBecomesFalse(() -> {
                    leftFireServo.setPower(0);
                    rightFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                });


        g2B.whenTrue(() -> flipper.setPosition(0.27))
                .whenFalse(() -> flipper.setPosition(0.4));

        g1Right.whenTrue(() -> turretMotor.setPower(0.4))
                .whenFalse(() -> turretMotor.setPower(0.0));

        g1Left.whenTrue(() -> turretMotor.setPower(-0.4))
                .whenFalse(() -> turretMotor.setPower(0.0));




    }
    @Override
    public void onUpdate() {
        hood.update();
        tracking.update();
        flywheel.update();
        BindingManager.update();
        drive.update();
        telemetry.update();
    }
}