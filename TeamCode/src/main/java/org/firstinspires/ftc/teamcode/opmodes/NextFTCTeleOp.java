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
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.*;

//Use the customized Drawing class, not the default
//import org.firstinspires.ftc.teamcode.pathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utilities.PrismLightValues;
import org.firstinspires.ftc.teamcode.vision.limelight.LimelightComponent;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import com.bylazar.telemetry.PanelsTelemetry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(name="teleop")
public class NextFTCTeleOp extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(NextFTCTeleOp.class);

    {
        addComponents(
                flywheel = new Flywheel(),
                drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                aimbot = new Aimbot(),
                turret = new Turret(),
                limelight = new Relocalization()
        );
    }
    Relocalization limelight;
    Turret turret;
    FieldCentricDrive drive;
    Aimbot aimbot;
    Flywheel  flywheel;
    TurretLights turretLights;
    private ElapsedTime looptime;
    private double highestLooptime = 0;
    private LimelightComponent limelightComponent;
    double FLYWHEEL_VEL;//= 1300; // IN RPM
    double HOOD_POS;
    double INTAKE_POWER = 0.9;
    private boolean FLYWHEEL_ON = false;
    private boolean isIntakeOn = false;
    private boolean isIntakeReversed = false;
    private boolean isTransferOn = false;
    int FLYWHEEL_STEP = 50;
    private DcMotor intakeMotor;
    private Servo flipper;
    private double FIRE_POWER = 0.9;
    private double slowModeMultiplier = 1;
    private CRServo leftFireServo, sideWheelServo;
    private GoBildaPrismDriver prism;
    Follower follower;
    public Alliance alliance;
    double botDistance;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        drive.setOffset(OpModeTransfer.currentPose.getHeading());
        follower.setStartingPose(OpModeTransfer.currentPose);
        follower.update();

        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        flipper = hardwareMap.get(Servo.class, "flipper");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretLights = new TurretLights(hardwareMap, telemetry);

        Drawing.init();

        alliance = OpModeTransfer.alliance;
        Button g1Back = button(() -> gamepad1.back);
        Button g2Back = button(() -> gamepad2.back);
        Button g2Options = button(() -> gamepad2.options);
        g2Options.whenBecomesTrue(() -> flywheel.resetHoodEncoder());
        g2Back.whenBecomesTrue(() -> turret.zeroTurret());
        g1Back.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    if (alliance == Alliance.BLUE) alliance = Alliance.RED;
                    else alliance = Alliance.BLUE;

                    turret.setAlliance(alliance);
                });
        looptime = new ElapsedTime();

    }
    @Override
    public void onWaitForStart() {
    }

    @Override
    public void onStartButtonPressed() {
        //follower.startTeleOpDrive();

        turret.setAlliance(alliance);
        aimbot.setAlliance(alliance);



        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        Button g2A = button(() -> gamepad2.a);
        Button g2RT = button(() -> gamepad2.right_trigger > 0.1);

        Button gUp = button(() -> gamepad2.dpad_up || gamepad1.dpad_up);
        Button gDown = button(() -> gamepad2.dpad_down || gamepad1.dpad_down);
        Button gUpOrDown = gUp.or(gDown);

        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);
        Button g1A = button(() -> gamepad1.a);
        Button g1B = button(() -> gamepad1.b);

        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);
        Button g2LT = button(() -> gamepad2.left_trigger > 0.1);
        Button g1RT = button(() -> gamepad1.right_trigger > 0.1);


        //Button g1X = button(() -> gamepad1.x);

        //g1X.whenBecomesTrue(() -> odoTurret.resetTurret());

        g1Right.whenBecomesTrue(() -> turret.turretStateForward());

        g1Left.whenBecomesTrue(() -> turret.turretStateBackward());

        g1RT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> slowModeMultiplier = 0.5)
                .whenBecomesFalse(() -> slowModeMultiplier = 1);



        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> FLYWHEEL_ON = true)
                .whenBecomesFalse(() -> FLYWHEEL_ON = false);


        g2A.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(INTAKE_POWER);
                    isIntakeOn = true;
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                    isIntakeOn = false;
                });

        g2LT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(-INTAKE_POWER);
                    sideWheelServo.setPower(-FIRE_POWER);
                    leftFireServo.setPower(-FIRE_POWER);
                    isIntakeReversed = true;
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                    sideWheelServo.setPower(0);
                    leftFireServo.setPower(0);
                    isIntakeReversed = false;
                });

        g1A.whenBecomesTrue(() -> {
            FrontAutoPaths.alliance = alliance;
            FrontAutoPaths.generatePaths(follower);
            Pose resetPose = new Pose(FrontAutoPaths.startingPose.getX(), FrontAutoPaths.startingPose.getY(), FrontAutoPaths.startAngle);
            follower.setPose(resetPose);
        });

        g2X.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    leftFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                    isTransferOn = true;
                })
                .whenBecomesFalse(() -> {
                    leftFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                    isTransferOn = false;
                });


        g2B.whenTrue(() ->
            flipper.setPosition(0.1)
            )
                .whenBecomesFalse(() -> flipper.setPosition(0.52));



        gUpOrDown.whenBecomesFalse(() -> {
                    flywheel.setHoodPower(0);
                    flywheel.resetHoodEncoder();
                });

        gUp.whenTrue(() -> flywheel.setHoodPower(0.2));
        gDown.whenTrue(() -> flywheel.setHoodPower(-0.2));

        g1B.whenBecomesTrue(() -> {
            drive.setOffset(follower.getHeading());

        });

        g1LT.whenBecomesTrue(() -> turretLights.redAlliance());

    }
    @Override
    public void onUpdate() {
        drive.update(follower.getHeading(), slowModeMultiplier);
        looptime.reset();
        follower.update();

        aimbot.setCurrentPose(follower.getPose());
        aimbot.update();
        FLYWHEEL_VEL = aimbot.getAimbotValues().velocity;
        HOOD_POS = aimbot.getAimbotValues().hoodPos;
        flywheel.setHoodGoalPos(HOOD_POS);
        if (FLYWHEEL_ON) {
            flywheel.setTargetVel(FLYWHEEL_VEL);
        } else {
            flywheel.setTargetVel(0);
        }

        /*double currentHeading = follower.getHeading();
        limelightComponent.update(currentHeading);
        if (limelightComponent.hasValidBotPose()) {
            int tagId = limelightComponent.getAprilTagId();
            boolean isGoalTag = (tagId >= 1 && tagId <= 2);
            if (isGoalTag) {
              Pose limelightPose = new Pose(limelightComponent.getRobotX(),limelightComponent.getRobotY(),limelightComponent.getRobotHeading());
            }
       }*/
        limelight.update();
        BindingManager.update();
        flywheel.update();

        turret.setCurrentPose(follower.getPose());
        turret.update();
        Drawing.drawOnlyCurrent(follower);
        //Drawing.drawDebug(follower);

        if (looptime.milliseconds() > highestLooptime) {
            highestLooptime = looptime.milliseconds();
        }

        telemetry.addData("looptime (ms)", looptime.milliseconds());
        telemetry.addData("highest looptime (ms)", highestLooptime);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Is Intake on: ", isIntakeOn);
        telemetry.addData("Is Intake Reversed: ", isIntakeReversed);
        telemetry.addData("Is Flywheel on: ", FLYWHEEL_ON);
        telemetry.addData("Is Transfer on: ", isTransferOn);
        telemetry.update();
    }
}
