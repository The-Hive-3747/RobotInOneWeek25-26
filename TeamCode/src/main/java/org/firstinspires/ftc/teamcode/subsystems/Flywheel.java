package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    MotorEx flywheel;
    static double GOBILDA_MOTOR_TICKS_PER_REVOLUTION = 28.0;
    static double correct, flywheelVel, targetVel, currentRPM;
    static double SECONDS_TO_MINUTES = 60.0;
    static double pastRPM = 0;
    static double shotCount = 0;
    ControlSystem flywheelController;
    Servo flipper;

    double autoTargetVel = 3500;
    double kV = 0.0004;
    double kP = 0.1;
    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        flywheel = new MotorEx("flywheel").reversed();
        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");

        // a control system is NextFTC's way to build.. control systems!
        flywheelController = ControlSystem.builder()
                .basicFF(kV) // we use a FeedForward (which pushes hard)
                .velPid(kP) // and a velocity PID (for fine tuning)
                .build(); // and build the control system!
        targetVel = 0; // setting a target velocity of 0 so that the robot doesnt blow up on start
    }

    // sets motor power DONT use this method normally, its not smart
    public void setPower(double power) {
        flywheel.setPower(power);
    }
    // gets motor power
    public double getPower() {
        return flywheel.getPower();
    }
    // gets motor velocity. needs to convert from TPS (ticks per second) to RPM
    public double getVel() {
        // NEED TO DO SOME MATH HERE!!!
        // getVelocity() returns a val in Ticks Per Second, so we divide by the Ticks Per Seconds by Ticks Per Revolution
        // Then we get Revolutions Per Second, so we need to multiply by 60 to convert it to Revolutions Per Minute (RPM)
        return -(flywheel.getVelocity() / GOBILDA_MOTOR_TICKS_PER_REVOLUTION)*SECONDS_TO_MINUTES;
    }

    // sets the target velocity! since we don't care abt the position of the flywheel, we can just set it to 0
    // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
    public void setTargetVel(double vel) {
        targetVel = vel;
        flywheelController.setGoal(new KineticState(0, targetVel));
    }


    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        flywheelVel = this.getVel();
        // correct is the motor power we need to set!
        correct = flywheelController.calculate( // calculate() lets us plug in current vals and outputs a motor power
                new KineticState(0, flywheelVel) // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
        );
        // setting constraints on our motor power so its not above 1 and not below 0
        if (correct > 0) {
            if (correct >= 1) {
                correct = 1;
            } else {
                correct = correct;
            }
        } else {
            correct = 0.4;
        }
        flywheel.setPower(correct); // set the motor power!
        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel);
        ActiveOpMode.telemetry().addData("balls shot", shotCount);
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
            .setIsDone(() -> true);;

    public Command shootAllThree = new LambdaCommand()
            .setStart(()->{
                ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
            })
            .setUpdate(() -> {
                currentRPM = this.getVel();
                if (pastRPM == 0) { pastRPM = currentRPM; }
                if (pastRPM - currentRPM >= 150) { shotCount++; }
                if (targetVel - currentRPM < 200) {
                    flipper.setPosition(0.27);
                } else {
                    flipper.setPosition(0.4);
                }
                pastRPM = currentRPM;
            })
            .setStop(interrupted -> {
                //this.setTargetVel(0);
                flipper.setPosition(0.4);
                pastRPM = 0;
                shotCount = 0;
            })
            .setIsDone(() -> (shotCount == 3));
}
