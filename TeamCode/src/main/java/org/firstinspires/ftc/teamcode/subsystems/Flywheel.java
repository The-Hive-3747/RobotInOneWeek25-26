package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;


// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    DcMotorEx flywheelLeft, flywheelRight;
    static double correct, flywheelVel, targetVel, currentRPM;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime colorTimer = new ElapsedTime();
    ControlSystem largeFlywheelPID;
    Servo light, flipper;
    Hood hood;

    double autoTargetVel = 1040;
    double kP = 0.55;
    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
        flywheelLeft = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelLeft");
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelRight");

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");

        hood = new Hood(flywheelLeft);
        hood.init();


        // a control system is NextFTC's way to build.. control systems!
        largeFlywheelPID = ControlSystem.builder()
                //.basicFF(kV) // we use a FeedForward (which pushes hard)
                .velPid(kP) // and a velocity PID (for fine tuning)
                .build(); // and build the control system!

        targetVel = 0; // setting a target velocity of 0 so that the robot doesnt blow up on start

        colorTimer.reset();
    }

    public void resetHoodEncoder() {
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    // sets motor power DONT use this method normally, its not smart
    public void setPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }
    // gets motor power
    public double getPower() {
        return flywheelRight.getPower();

    }
    // gets motor velocity. needs to convert from TPS (ticks per second) to RPM
    public double getVel() {
        return (flywheelRight.getVelocity()); // GOBILDA_TICKS_PER_REVOLUTION)*SECONDS_TO_MINUTES;
    }

    // sets the target velocity! since we don't care abt the position of the flywheel, we can just set it to 0
    // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
    public void setTargetVel(double vel) {
        targetVel = vel;
        largeFlywheelPID.setGoal(new KineticState(0, targetVel));
    }




    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        flywheelVel = this.getVel();

        // correct is the motor power we need to set!

        correct = largeFlywheelPID.calculate( // calculate() lets us plug in current vals and outputs a motor power
                new KineticState(0, flywheelVel) // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
        );


        // setting constraints on our motor power so its not above 1 and not below 0
        if (targetVel != 0) {
            if (correct > 0) {
                if (correct >= 1) {
                    correct = 1;
                }
            } else {
                correct = 0.7 * targetVel/1300;
            }
        } else {
            correct = 0;
        }
        this.setPower(correct); // set the motor power!

        hood.update();

        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel);
        //ActiveOpMode.telemetry().addData("balls shot", shotCount);
        //ActiveOpMode.telemetry().addData("rightVel", flywheelRight.getVelocity());
    }

    // HOOD FUNCTIONS
    public double getHoodPos() {
        return hood.getHoodPosition();
    }

    public void setHoodGoalPos(double pos) {
        hood.setGoal(pos);
    }

    public double getHoodGoal() {
        return hood.getGoal();
    }

    public void setHoodPower(double pow) {
        hood.setHoodPower(pow);
    }

    public void enableHoodPid() {
        hood.enableHoodPID();
    }



    public Command startFlywheel = new InstantCommand(
            () -> this.setTargetVel(autoTargetVel)
    );
    public Command stopFlywheel = new InstantCommand(
            () -> this.setTargetVel(0)
    );

    public Command resetShotTimer = new InstantCommand(
            () ->  shotTimer.reset()
    );

    public Command shootAllThree = new LambdaCommand()
            .setStart(() -> {
                ActiveOpMode.telemetry().addLine("flywheel is shootinggg");

            })
            .setUpdate(() -> {
                currentRPM = this.getVel();
                if (Math.abs(targetVel - currentRPM) < 200) {
                    light.setPosition(0.67);
                    flipper.setPosition(0.1);
                } else {
                    flipper.setPosition(0.52);
                }
            })
            .setStop(interrupted -> {
                flipper.setPosition(0.52);
                light.setPosition(0.388);
            })
            .setIsDone(() -> (shotTimer.seconds() > 2)); // TODO: CHANGE THIS TO THREE
}
