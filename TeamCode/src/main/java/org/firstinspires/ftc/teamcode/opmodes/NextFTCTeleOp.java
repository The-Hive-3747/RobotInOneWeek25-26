package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.*;

import org.firstinspires.ftc.teamcode.helpers.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretUsingOdo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name="teleop")
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                new SubsystemComponent(Hood.INSTANCE),
                flywheel = new Flywheel(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                odoTurret = new TurretUsingOdo()
        );
    }
    Hood hood;
    TurretUsingOdo odoTurret;

    Flywheel flywheel;
    private ElapsedTime looptime;
    private double highestLooptime = 0;
    double FLYWHEEL_VEL = 1300; // IN RPM
    double INTAKE_POWER = 0.9;
    private double HOOD_POSITION = 0.0;
    private DcMotor intakeMotor;
    private Servo flipper, light;
    private double FIRE_POWER = 0.9;
    private CRServo leftFireServo;
    private CRServo rightFireServo;
    private CRServo sideWheelServo;
    Follower follower;
    private double color;
    public boolean isRed;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(OpModeTransfer.currentPose);
        follower.update();

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
        looptime = new ElapsedTime();
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

        follower.startTeleOpDrive();
        odoTurret.allowTurret();
        odoTurret.setAlliance(isRed);
        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g2A = button(() -> gamepad2.a);

        Button gUp = button(() -> gamepad2.dpad_up || gamepad1.dpad_up);
        Button gDown = button(() -> gamepad2.dpad_down || gamepad1.dpad_down);
        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Up = button(() -> gamepad1.dpad_up);
        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);
        Button g1RT = button(() -> gamepad1.right_trigger > 0.1);
        Button g1Down = button(() -> gamepad1.dpad_down);
        Button gUpOrDown = gUp.or(gDown);

        Button g1A = button(() -> gamepad1.a);
        Button g1B = button(() -> gamepad1.b);

        g1Right.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    odoTurret.setTurretAngle(0);
                        })
                .whenBecomesFalse(() -> {
                    odoTurret.allowTurret();
                });
        g1RT.toggleOnBecomesTrue()
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
        g1LT.whenTrue(() -> {flipper.setPosition(0.1); color=0.67;})
                .whenFalse(() -> {flipper.setPosition(0.52); color=0.388;});

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



        gUpOrDown.whenBecomesFalse(() -> {
                    hood.INSTANCE.setGoal(hood.INSTANCE.getHoodPosition());
                    hood.INSTANCE.setHoodPower(0);
                });
        gUp.whenTrue(() -> hood.INSTANCE.setHoodPower(0.1));
        gDown.whenTrue(() -> hood.INSTANCE.setHoodPower(-0.1));

        g1A.whenBecomesTrue(() -> {
            hood.INSTANCE.setGoal(hood.INSTANCE.getGoal() + 250);
        });
        g1B.whenBecomesTrue(() -> {
            hood.INSTANCE.setGoal(hood.INSTANCE.getGoal() - 250);
        });


    }
    @Override
    public void onUpdate() {
        looptime.reset();
        light.setPosition(color);
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();
        flywheel.update();
        BindingManager.update();
        odoTurret.setCurrentPose(follower.getPose());
        odoTurret.update();
        if (looptime.milliseconds() > highestLooptime) {
            highestLooptime = looptime.milliseconds();
        }
        telemetry.addData("looptime (ms)", looptime.milliseconds());
        telemetry.addData("highest looptime (ms)", highestLooptime);
        telemetry.update();
    }
}
