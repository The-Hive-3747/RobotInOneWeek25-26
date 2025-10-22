package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.interpolators.FirstOrderEMAInterpolator;

@TeleOp
public class intakeTest extends LinearOpMode {
    private DcMotor intakeMotor;
    private CRServo leftFireServo;
    private CRServo rightFireServo;
    private DcMotor flywheelMotor;
    private double INTAKE_POWER = 0.5;
    private double INTAKE_POWER_STEP = 0.05;
    private double FIRE_POWER = 0.3;
    private double FIRE_POWER_STEP = 0.05;
    private double FLYWHEEL_POWER = 0.6;
    private double FLYWHEEL_POWER_STEP = 0.05;
    private boolean isIntakeOn = false;
    private boolean wasAPressed = false;
    private boolean wasX2Pressed = false;
    private boolean wasB2Pressed = false;
    private boolean wasB1Pressed = false;
    private boolean isFireServoOn = false;
    private boolean wasA2Pressed = false;
    private boolean wasY2Pressed = false;
    private boolean wasY1Pressed = false;
    private boolean isFlywheelOn = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        rightFireServo = hardwareMap.get(CRServo.class, "right_firewheel");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()){
            //this is for intake
            if(gamepad1.a && !wasAPressed){
                if (!isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                    isIntakeOn = true;
                }else{
                    intakeMotor.setPower(0);
                    isIntakeOn = false;
                }
                wasAPressed = true;
            }
            if(!gamepad1.a && wasAPressed){
                wasAPressed = false;
            }
            if(gamepad2.x && !wasX2Pressed){
                wasX2Pressed = true;
                INTAKE_POWER = INTAKE_POWER  - INTAKE_POWER_STEP;
                if(INTAKE_POWER < -1.0){
                    INTAKE_POWER = -1.0;
                }
                if(isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                }
            }
            if(!gamepad2.x && wasX2Pressed){
                wasX2Pressed = false;
            }
            if(gamepad2.b && !wasB2Pressed){
                wasB2Pressed = true;
                INTAKE_POWER = INTAKE_POWER  + INTAKE_POWER_STEP;
                if(INTAKE_POWER > 1.0){
                    INTAKE_POWER = 1.0;
                }
                if(isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                }
            }
            if(!gamepad2.b && wasB2Pressed){
                wasB2Pressed = false;
            }
            //this is for fireservos
            if(gamepad1.b && !wasB1Pressed){
                if (!isFireServoOn){
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                    isFireServoOn = true;
                }else{
                    leftFireServo.setPower(0);
                    rightFireServo.setPower(0);
                    isFireServoOn = false;
                }
                wasB1Pressed = true;
            }
            if(!gamepad1.b && wasB1Pressed){
                wasB1Pressed = false;
            }
            if(gamepad2.a && !wasA2Pressed){
                wasA2Pressed = true;
                FIRE_POWER = FIRE_POWER - FIRE_POWER_STEP;
                if(FIRE_POWER < -1.0){
                    FIRE_POWER = -1.0;
                }
                if(isFireServoOn){
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                }
            }
            if(!gamepad2.a && wasA2Pressed){
                wasA2Pressed = false;
            }
            if(gamepad2.y && !wasY2Pressed){
                wasY2Pressed = true;
                FIRE_POWER = FIRE_POWER  + FIRE_POWER_STEP;
                if(FIRE_POWER > 1.0){
                    FIRE_POWER = 1.0;
                }
                if(isFireServoOn){
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                }
            }
            if(!gamepad2.y && wasY2Pressed){
                wasY2Pressed = false;
            }
            //this is for the flywheel
            if(gamepad1.y && !wasY1Pressed){
                if (!isFlywheelOn){
                    flywheelMotor.setPower(FLYWHEEL_POWER);
                    isFlywheelOn = true;
                }else{
                    flywheelMotor.setPower(0);
                    isFlywheelOn = false;
                }
                wasY1Pressed = true;
            }
            if(!gamepad1.y && wasY1Pressed){
                wasY1Pressed = false;
            }
            if(gamepad2.dpad_left && !wasDpadLeftPressed){
                wasDpadLeftPressed = true;
                FLYWHEEL_POWER = FLYWHEEL_POWER - FLYWHEEL_POWER_STEP;
                if(FLYWHEEL_POWER < -1.0){
                    FLYWHEEL_POWER_STEP = -1.0;
                }
                if(isFlywheelOn){
                    flywheelMotor.setPower(FLYWHEEL_POWER);
                }
            }
            if(!gamepad2.dpad_left && wasDpadLeftPressed){
                wasDpadLeftPressed = false;
            }
            if(gamepad2.dpad_right && !wasDpadRightPressed){
                wasDpadRightPressed = true;
                FLYWHEEL_POWER = FLYWHEEL_POWER  + FLYWHEEL_POWER_STEP;
                if(FLYWHEEL_POWER > 1.0){
                    FLYWHEEL_POWER = 1.0;
                }
                if(isFlywheelOn){
                    flywheelMotor.setPower(FLYWHEEL_POWER);
                }
            }
            if(!gamepad2.dpad_right && wasDpadRightPressed){
                wasDpadRightPressed = false;
            }
            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Press Gamepad 2 X to decrease the intake power by 5%", "");
            telemetry.addData("Press Gamepad 2 B to increase the intake power by 5%", "");
            telemetry.addData("Press Gamepad 1 A to toggle the intake","");
            telemetry.addData("Fire Servos Power", FIRE_POWER);
            telemetry.addData("Press Gamepad 2 A to decrease the fire servos power by 5%", "");
            telemetry.addData("Press Gamepad 2 Y to increase the fire servos power by 5%", "");
            telemetry.addData("Press Gamepad 1 B to toggle the fire servos","");
            telemetry.addData("Flywheel Power", FLYWHEEL_POWER);
            telemetry.addData("Press Gamepad 2 Dpad Left to decrease the flywheel power by 5%", "");
            telemetry.addData("Press Gamepad 2 Dpad Right increase the flywheel power by 5%", "");
            telemetry.addData("Press Gamepad 1 Y to toggle the flywheel","");
            telemetry.update();
        }
    }
}
