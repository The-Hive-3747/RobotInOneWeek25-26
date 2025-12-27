package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Turret implements Component {
    DcMotorEx turret;
    TouchSensor limitSwitch;
    Pose currentPose;
    ElapsedTime resetTimer = new ElapsedTime();
    boolean resetStarted = false;
    boolean hasBeenReset = false;
    private double LEFT_STOP_DEG = -102.0;
    private double RESET_TIME_MS = 2000;
    private double RESET_TO_ZERO_MS = 4000;
    public enum turretState {
        OFF,
        FORWARD,
        AUTO,
        ZEROING
    }
    turretState currentState = turretState.AUTO;
    Alliance alliance;
    private double goalAngle, goalX, goalY, turretPower, turretGoal, heading;
    private KineticState ZERO_ANGLE = new KineticState(0);

    private double TURRET_PID_KP = 0.058, TURRET_PID_KD = 0.01;
    private final double LEFT_TURRET_LIMIT = -100, RIGHT_TURRET_LIMIT = 130;
    private double TURRET_POWER_LIMIT = 0.9, TURRET_ANGLE_DEADZONE = 0.5;
    private int TURRET_TICKS_TO_ANGLES = 90/6100;
    ControlSystem turretPID;

    @Override
    public void preInit() {
        limitSwitch = ActiveOpMode.hardwareMap().get(TouchSensor.class, "limitSwitch");
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        currentState = turretState.AUTO;
        turretPID = ControlSystem.builder()
                .posPid(TURRET_PID_KP, 0, TURRET_PID_KD)
                .build();
        turretGoal = 0;
    }


    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetTurret(){
        currentState = turretState.ZEROING;
        resetStarted = false;
        resetTimer.reset();
    }

    public void update() {
        if (currentState == turretState.AUTO) {
            turretPID.setGoal(getAutoAimGoalAngle());
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
        } else if (currentState == turretState.FORWARD) {
            turretPID.setGoal(ZERO_ANGLE);
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
        } else {
            // this is when the TurretState is Off
            turretPower = 0;
        }

        if (limitSwitch.isPressed()) {
            if (!hasBeenReset) {
                this.zeroTurret();
                hasBeenReset = true;
            }
        } else if (hasBeenReset) {
            hasBeenReset = false;
        }

        // limit the turret power to our Turret Power Limit
        turretPower = Math.min(TURRET_POWER_LIMIT, turretPower);

        // Clamp goal so that if we're within the deadzone, we dont waste power
        if (Math.abs(getTurretAngle() - turretGoal) < TURRET_ANGLE_DEADZONE) {
            turretPower = 0;
        }

        // Need to do this because our encoder and motor are reversed
        // TODO: FLIP ENCODER MECHANICALLY SO THAT THEY MATCH UP
        turretPower *= -1;


        turret.setPower(turretPower);

        ActiveOpMode.telemetry().addData("turret state", currentState);
        ActiveOpMode.telemetry().addData("turret goal", turretPID.getGoal().component1());
        //ActiveOpMode.telemetry().addData("turret power", turretPower);
        ActiveOpMode.telemetry().addData("turret angle", this.getTurretAngle());
    }

    /**
     * @return: KineticState of goal, for auto-aim.
     */
    public KineticState getAutoAimGoalAngle() {
        if (currentPose != null) {
            goalAngle = -Math.atan2((goalY - this.currentPose.getY()), (goalX - this.currentPose.getX())); // IN RADS
            turretGoal = normalizeAngle(goalAngle + this.currentPose.getHeading() + Math.PI);
            return new KineticState(this.putInTurretLimits(Math.toDegrees(turretGoal)));
        } else {
            return ZERO_ANGLE;
        }
    }


    /**
     *
     * @param goal: goal that you are putting within turret limits
     * @return goal: which is within turret limits
     */
    public double putInTurretLimits(double goal) {
        if (goal > RIGHT_TURRET_LIMIT || goal < LEFT_TURRET_LIMIT) {
            if (goal > RIGHT_TURRET_LIMIT) {
                goal = RIGHT_TURRET_LIMIT;
            } else {
                goal = LEFT_TURRET_LIMIT;
            }
        }
        return goal;
    }

    /**
     *
     * @param pose: sets the current pose, used for auto-aim calculations
     *            NEEDS TO BE DONE EVERY LOOP
     */
    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    /**
     *
     * @param goal: goal angle for turret to face
     */
    public void setTurretAngle(double goal) {
        currentState = turretState.FORWARD;
        turretPID.setGoal(new KineticState(goal));
    }


    public turretState getTurretState() {
        return currentState;
    }


    /**
     *
     * @return turret angle after converting from ticks
     * IN DEGREES
     */
    public double getTurretAngle() {
        return (double) (turret.getCurrentPosition() * 90) /6100;
    }

    public static double normalizeAngle(double angleRad) {
        // Ensure the angle is within the (-π, π] range
        angleRad %= (2 * Math.PI); // Take the modulo with 2π
        if (angleRad > Math.PI) {
            angleRad -= (2 * Math.PI); // Subtract 2π if greater than π
        } else if (angleRad <= -Math.PI) {
            angleRad += (2 * Math.PI); // Add 2π if less than or equal to -π
        }
        return angleRad;
    }

    /**
     *
     * @param all: alliance we're on
     *           sets the goal (physical) values for red & blue
     */
    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 140;
            goalY = 144;
        } else {
            goalX = 4;
            goalY = 144;
        }
    }

    public Command setTurretAuto = new InstantCommand(
            () -> currentState = turretState.AUTO
    );
    public Command setTurretOff = new LambdaCommand()
            .setStart(() -> {
                currentState = turretState.OFF;
            })
            .setIsDone(() -> true);
    public Command setTurretForward = new LambdaCommand()
            .setStart(() -> {
                currentState = turretState.FORWARD;
            })
            .setIsDone(() -> true);

    public void turretStateForward() {
        switch (currentState) {
            case AUTO:
                this.currentState = turretState.FORWARD;
                break;
            case FORWARD:
                this.currentState = turretState.OFF; break;
            case OFF:
                this.currentState = turretState.AUTO; break;
        }
    }

    public void turretStateBackward() {
        switch (currentState) {
            case AUTO:
                this.currentState = turretState.OFF;
                break;
            case FORWARD:
                this.currentState = turretState.AUTO;
                break;
            case OFF:
                this.currentState = turretState.FORWARD;
                break;
        }
    }

}
