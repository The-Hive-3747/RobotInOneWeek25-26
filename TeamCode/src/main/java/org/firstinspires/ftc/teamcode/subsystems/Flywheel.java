package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;


// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    MotorEx flywheelLeft, flywheelRight;
    static double GOBILDA_TICKS_PER_REVOLUTION = 4096;
    static double correct, flywheelVel, targetVel, currentRPM, targetHoodPos, correctHood, oldPos, currentPos, color, hoodPos;
    static double SECONDS_TO_MINUTES = 60.0;
    MotorGroup flywheels;
    static double pastRPM = 0;
    static double shotCount = 0;
    static boolean shotCountJustChanged = false;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime colorTimer = new ElapsedTime();
    ControlSystem flywheelController, hoodController;
    Servo light;
    CRServo hood;
    Servo flipper;

    double autoTargetVel = 1100;
    double kV = 0.0004;
    double kP = 0.1;
    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        flywheelLeft = new MotorEx("flywheelLeft").reversed();
        flywheelRight = new MotorEx("flywheelRight");
        flywheelRight.zero();

        flywheels = new MotorGroup(flywheelLeft, flywheelRight);
        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");

        hoodController = ControlSystem.builder()
                .posPid(0.5)
                .build();
        targetHoodPos = 0;

        // a control system is NextFTC's way to build.. control systems!
        flywheelController = ControlSystem.builder()
                .basicFF(kV) // we use a FeedForward (which pushes hard)
                .velPid(kP) // and a velocity PID (for fine tuning)
                .build(); // and build the control system!
        targetVel = 0; // setting a target velocity of 0 so that the robot doesnt blow up on start

        colorTimer.reset();
    }

    public void setHoodPos(double pos) {
        targetHoodPos = pos;
        hoodController.setGoal(new KineticState(targetHoodPos));
    }
    public double getHoodPos() {
        return (flywheelLeft.getCurrentPosition()+21)/37;//2000;
    }

    // sets motor power DONT use this method normally, its not smart
    public void setPower(double power) {
        flywheels.setPower(power);
    }
    // gets motor power
    public double getPower() {
        return flywheels.getPower();

    }
    // gets motor velocity. needs to convert from TPS (ticks per second) to RPM
    public double getVel() {
        // NEED TO DO SOME MATH HERE!!!
        // getVelocity() returns a val in Ticks Per Second, so we divide by the Ticks Per Seconds by Ticks Per Revolution
        // Then we get Revolutions Per Second, so we need to multiply by 60 to convert it to Revolutions Per Minute (RPM)
        return (flywheelRight.getVelocity()); // GOBILDA_TICKS_PER_REVOLUTION)*SECONDS_TO_MINUTES;
    }

    // sets the target velocity! since we don't care abt the position of the flywheel, we can just set it to 0
    // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
    public void setTargetVel(double vel) {
        targetVel = vel;
        flywheelController.setGoal(new KineticState(0, targetVel));
    }

    public void rainbowLight(boolean on) {
        if (on) {
            if (colorTimer.milliseconds() > 150) {
                color += 0.001;
                if (color>0.772) {
                    color = 0.279;
                }
                colorTimer.reset();
            }
            light.setPosition(color);
        } else {
            light.setPosition(0);
        }
    }


    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        flywheelVel = this.getVel();
        //hoodPos = this.getHoodPos();

        /*correctHood = hoodController.calculate(
            new KineticState(hoodPos)
        );*/
        //hood.setPower(Math.abs(correctHood) > 0.2 ? correctHood : 0);
        // correct is the motor power we need to set!
        correct = flywheelController.calculate( // calculate() lets us plug in current vals and outputs a motor power
                new KineticState(0, flywheelVel) // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
        );
        // setting constraints on our motor power so its not above 1 and not below 0
        if (targetVel != 0) {
            if (correct > 0) {
                if (correct >= 1) {
                    correct = 1;
                }
            } else {
                correct = 0.8;
            }
        } else {
            correct = 0;
        }
        flywheels.setPower(correct); // set the motor power!
        
        
        this.rainbowLight(true);
        

        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel);
        ActiveOpMode.telemetry().addData("balls shot", shotCount);
        ActiveOpMode.telemetry().addData("hood pos", hoodPos);
        ActiveOpMode.telemetry().addData("rightVel", -flywheelRight.getVelocity());
    }

    public Command startFlywheel = new LambdaCommand()
            .setStart(() -> {
                this.setTargetVel(autoTargetVel);
                    })
            .setIsDone(() -> true);;
    public Command stopFlywheel = new LambdaCommand()
            .setStart(() -> {
                this.setTargetVel(0);
            })
            .setIsDone(() -> true);

    public Command resetShotTimer = new LambdaCommand()
            .setStart(() -> {
                shotTimer.reset();
            })
            .setIsDone(() -> true);

    public Command shootAllThree = new LambdaCommand()
            .setStart(() -> {
                ActiveOpMode.telemetry().addLine("flywheel is shootinggg");

            })
            .setUpdate(() -> {
                currentRPM = this.getVel();
                if ((targetVel - currentRPM) < 80) {
                    flipper.setPosition(0.1);
                    this.rainbowLight(true);
                } else {
                    flipper.setPosition(0.52);
                }
            })
            .setStop(interrupted -> {
                flipper.setPosition(0.52);
            })
            .setIsDone(() -> (shotTimer.seconds() > 3.2)); // TODO: CHANGE THIS TO THREE
}
