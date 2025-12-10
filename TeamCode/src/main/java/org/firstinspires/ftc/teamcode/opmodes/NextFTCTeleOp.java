package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.conditionals.SwitchCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.*;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretUsingOdo;
import org.firstinspires.ftc.teamcode.subsystems.TurretUsingOdo.turretState;
import org.firstinspires.ftc.teamcode.vision.limelight.LimelightComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name="teleop")
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                flywheel = new Flywheel(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                odoTurret = new TurretUsingOdo()
        );
    }
    TurretUsingOdo odoTurret;
    Flywheel flywheel;
    private ElapsedTime looptime;
    private double highestLooptime = 0;
    private LimelightComponent limelightComponent;
    double FLYWHEEL_VEL = 1300;//= 1300; // IN RPM
    double INTAKE_POWER = 0.9;
    int FLYWHEEL_STEP = 50;
    private DcMotor intakeMotor;
    private Servo flipper, light;
    private double FIRE_POWER = 0.9;
    private double SLOW_MODE = 1;
    private CRServo leftFireServo, sideWheelServo;
    Follower follower;
    private double color;
    public Alliance alliance;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(OpModeTransfer.currentPose);
        follower.update();

        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        flipper = hardwareMap.get(Servo.class, "flipper");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
        alliance = OpModeTransfer.alliance;
        Button g1Back = button(() -> gamepad1.back);
        g1Back.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    if (alliance == Alliance.BLUE) alliance = Alliance.RED;
                    else alliance = Alliance.BLUE;

                    odoTurret.setAlliance(alliance);
                });
        looptime = new ElapsedTime();
    }
    @Override
    public void onWaitForStart() {
        if (alliance == Alliance.RED) {
            light.setPosition(0.279);
        } else {
            light.setPosition(0.611);
        }

    }

    @Override
    public void onStartButtonPressed() {

        follower.startTeleOpDrive();

        odoTurret.setTurretAuto.schedule();
        odoTurret.setAlliance(alliance);


        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g2A = button(() -> gamepad2.a);

        Button gUp = button(() -> gamepad2.dpad_up || gamepad1.dpad_up);
        Button gDown = button(() -> gamepad2.dpad_down || gamepad1.dpad_down);
        Button gUpOrDown = gUp.or(gDown);

        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);

        Button g2LT = button(() -> gamepad2.left_trigger > 0.1);

        Button g1RT = button(() -> gamepad1.right_trigger > 0.1);

        Button g1B = button(() -> gamepad1.b);

        g1Right.whenBecomesTrue(() -> odoTurret.turretStateForward.schedule());

        g1Left.whenBecomesTrue(() -> odoTurret.turretStateBackward.schedule());

        /*
        g1RT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    FLYWHEEL_VEL = 1300;
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
        g1RB.whenBecomesTrue(() -> {
            if (FLYWHEEL_VEL >= 1600) {
                FLYWHEEL_VEL = 1600;
                flywheel.setTargetVel(FLYWHEEL_VEL);

            }
            else{
                FLYWHEEL_VEL = FLYWHEEL_VEL + FLYWHEEL_STEP;
                flywheel.setTargetVel(FLYWHEEL_VEL);

            }
        });
        g1LB.whenBecomesTrue(() -> {
            if (FLYWHEEL_VEL <= 100) {
                FLYWHEEL_VEL = 0;
                flywheel.setTargetVel(FLYWHEEL_VEL);
            }
            else {
                FLYWHEEL_VEL = FLYWHEEL_VEL - FLYWHEEL_STEP;
                flywheel.setTargetVel(FLYWHEEL_VEL);
            }
        }); */

        g1RT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> SLOW_MODE = 0.5)
                .whenBecomesFalse(() -> SLOW_MODE = 1);



        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> flywheel.setTargetVel(FLYWHEEL_VEL))
                .whenBecomesFalse(() -> flywheel.setTargetVel(0.0));


        g2X.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(INTAKE_POWER);
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                });

        g2LT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(-INTAKE_POWER);
                    sideWheelServo.setPower(-FIRE_POWER);
                    leftFireServo.setPower(-FIRE_POWER);
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                    sideWheelServo.setPower(0);
                    leftFireServo.setPower(0);
                });

        g2A.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    leftFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                })
                .whenBecomesFalse(() -> {
                    leftFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                });



        g2B.whenTrue(() -> { flipper.setPosition(0.1); color=0.67; })
                .whenBecomesFalse(() -> { flipper.setPosition(0.52); color=0.388; });



        gUpOrDown.whenBecomesFalse(() -> {
                    flywheel.setHoodGoalPos(flywheel.getHoodPos());
                    flywheel.setHoodPower(0);
                });

        gUp.whenTrue(() -> flywheel.setHoodPower(0.2));
        gDown.whenTrue(() -> flywheel.setHoodPower(-0.2));


        g1B.whenBecomesTrue(() -> {
            follower.setPose(
                    new Pose(follower.getPose().getX(), follower.getPose().getY(), 0)
            );
        });


    }
    @Override
    public void onUpdate() {
        looptime.reset();
        light.setPosition(color);

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_MODE,
                -gamepad1.left_stick_x * SLOW_MODE,
                -gamepad1.right_stick_x * SLOW_MODE,
                false
        );
        follower.update();

        double currentHeading = follower.getHeading();
        limelightComponent.update(currentHeading);
        if (limelightComponent.hasValidBotPose()) {
            int tagId = limelightComponent.getAprilTagId();
            boolean isGoalTag = (tagId >= 1 && tagId <= 2);
            if (isGoalTag) {
              Pose limelightPose = new Pose(limelightComponent.getRobotX(),limelightComponent.getRobotY(),limelightComponent.getRobotHeading());
            }
       }

        BindingManager.update();
        flywheel.update();

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
