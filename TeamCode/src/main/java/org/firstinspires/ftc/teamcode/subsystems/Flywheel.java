package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    MotorEx flywheel;
    static double GOBILDA_MOTOR_TICKS_PER_REVOLUTION = 28.0;
    static double SECONDS_TO_MINUTES = 60.0;
    ControlSystem flywheelController;
    static double targetVel;
    double kV = 0.0004;
    @Override public void postInit() { // this runs AFTER the init, it runs just once
        flywheel = new MotorEx("flywheel");

        // a control system is NextFTC's way to build.. control systems!
        flywheelController = ControlSystem.builder()
                .basicFF(kV) // we use a FeedForward (which pushes hard)
                .velPid(0.1) // and a velocity PID (for fine tuning)
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
        return (flywheel.getVelocity() / GOBILDA_MOTOR_TICKS_PER_REVOLUTION)*SECONDS_TO_MINUTES;
    }

    // sets the target velocity! since we don't care abt the position of the flywheel, we can just set it to 0
    // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
    public void setTargetVel(double vel) {
        targetVel = vel;
        flywheelController.setGoal(new KineticState(0, targetVel));
    }


    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        // correct is the motor power we need to set!
        double correct = flywheelController.calculate( // calculate() lets us plug in current vals and outputs a motor power
                new KineticState(0, this.getVel()) // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
        );
        // setting constraints on our motor power so its not above 1 and not below 0
        if (correct > 0) {
            if (correct >= 1) {
                correct = 1;
            } else {
                correct = correct;
            }
        } else {
            correct = 0;
        }
        flywheel.setPower(correct); // set the motor power!
        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", this.getVel());
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel);
    }
}
