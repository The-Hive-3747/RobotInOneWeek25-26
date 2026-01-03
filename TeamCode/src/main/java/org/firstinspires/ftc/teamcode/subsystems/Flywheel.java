package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    DcMotorEx flywheelBottom, flywheelTop, intakeMotor;
    static double correct, flywheelVel, targetVel, currentRPM;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime colorTimer = new ElapsedTime();
    ControlSystem largeFlywheelPID;
    Servo flipper;
    Hood hood;

    double autoTargetVel = 1040;
    public static double kP = 0.06;//0.02//0.55
    public static double kS = 0.00011;
    public static double kD = 0.0;
    double targetAdjust = 0;
    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        //this needs to be forward in order to use the hood PID. correction is in set power
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        flywheelBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelTop = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelTop");
        flywheelTop.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flywheelBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");
        hood = new Hood(intakeMotor);
        hood.init();


        // a control system is NextFTC's way to build.. control systems!
        largeFlywheelPID = ControlSystem.builder()
                //.basicFF(0, 0, kS) // we use a FeedForward (which pushes hard)
                .velPid(kP) // and a velocity PID (for fine tuning)
                .build(); // and build the control system!

        targetVel = 0; // setting a target velocity of 0 so that the robot doesnt blow up on start

        colorTimer.reset();
    }

    /**
     * self-explanatory, resets the hood encoder
     */
    public void resetHoodEncoder() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void increase(){
        targetAdjust += 5;
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    public void decrease(){
        targetAdjust -= 5;
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    /**
     *
     * @param power sets motor power. DONT use this method normally, its not smart
     */
    public void setPower(double power) {
        //this is to correct the flywheel direction
        flywheelBottom.setPower(power);
        flywheelTop.setPower(power);
    }


    /**
     *
     * @return motor power
     */
    public double getPower() {
        return flywheelBottom.getPower();

    }


    /**
     *
     * @return gets flywheel motor velocity
     */
    public double getVel() {
        return (flywheelBottom.getVelocity());
    }


    /**
     *
     * @param vel: sets target velocity
     */
    public void setTargetVel(double vel) {
        double targetV = vel;
        targetVel = vel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
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
                if (correct >= 0.9) {
                    correct = 0.9;
                }
            } else {
                correct = 0.7 * targetVel/1300;
            }
        } else {
            correct = 0;
        }

        /*if (correct > 0) {
            if (correct >= 0.9) {
                correct = 0.9;
            }
        }
        if (correct <= 0){
            correct = 0;
        }
        */

        this.setPower(correct); // set the motor power!

        hood.update();

        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel);
    }


    // q: why does the hood own the flywheel?
    // a: the hood encoder uses the left flywheel's encoder port,
    // so everything becomes easier when the hood is owned by the flywheel

    // HOOD METHODS
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
            .setStart(() ->
                    ActiveOpMode.telemetry().addLine("flywheel is shootinggg")
            )
            .setUpdate(() -> {
                currentRPM = this.getVel();
                if (Math.abs(targetVel - currentRPM) < 200) {
                    flipper.setPosition(0.1);
                } else {
                    flipper.setPosition(0.52);
                }
            })
            .setStop(interrupted -> {
                flipper.setPosition(0.52);
            })
            .setIsDone(() -> (shotTimer.seconds() > 2));
            }
