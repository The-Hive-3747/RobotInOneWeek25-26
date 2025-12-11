package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.SwitchCommand;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class TurretUsingOdo implements Component {
    DcMotorEx turret;
    Pose currentPose;
    public enum turretState {
        OFF,
        FORWARD,
        AUTO
    }
    turretState currentState = turretState.AUTO;
    Alliance alliance;
    double goalAngle, goalX, goalY, turretPower, turretGoal;
    ControlSystem turretPID;

    @Override
    public void preInit() {
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");

        turretPID = ControlSystem.builder()
                .posPid(0.04, 0, 0.01)
                .build();
        turretGoal = 0;
    }

    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void update() {
        if (currentState == turretState.AUTO) {
            getGoalAngle();
        } else if (currentState == turretState.FORWARD) {
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
        } else {
            turretPower = 0;
        }
        if (turretPower > 0.8) {
            turretPower = 0.8;
        } else if (Math.abs(getTurretAngle() - turretGoal) < 2) {
            turretPower = 0;
        }

        turretPower *= -1;

        if (currentPose.getY() > 70) {
            turret.setPower(turretPower);
        } else {
            turret.setPower(0);
        }

        ActiveOpMode.telemetry().addData("turret goal", turretPID.getGoal().component1());
        ActiveOpMode.telemetry().addData("turret power", turretPower);
        ActiveOpMode.telemetry().addData("turret angle", this.getTurretAngle());
    }

    public void getGoalAngle() {
        if (currentPose != null) {
            goalAngle = -Math.atan2((goalY - this.currentPose.getY()), (goalX - this.currentPose.getX())); // IN RADS
            turretGoal = normalizeAngle(goalAngle + currentPose.getHeading());
            turretPID.setGoal(new KineticState(this.putInTurretLimits(Math.toDegrees(turretGoal))));
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
        }
    }

    public double putInTurretLimits(double goal) {
        if (goal > 60 || goal < -60) { //insanely horribly code
            if (goal > 60) {
                goal = 60;
            } else {
                goal = -60;
            }
        }
        return goal;
    }

    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    public void setTurretAngle(double goal) {
        currentState = turretState.FORWARD;
        turretPID.setGoal(new KineticState(goal));
    }


    public turretState getTurretState() {
        return currentState;
    }

    public double getTurretAngle() {
        return (turret.getCurrentPosition()*90)/6100;
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

    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 144;
            goalY = 144;
        } else {
            goalX = 0;
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
                setTurretAngle(0);
            })
            .setIsDone(() -> true);

    public Command turretStateForward = new InstantCommand(() -> {
        switch (currentState) {
            case AUTO:
                this.setTurretForward.schedule(); break;
            case FORWARD:
                this.setTurretOff.schedule(); break;
            case OFF:
                this.setTurretAuto.schedule(); break;
        }
    });

    public Command turretStateBackward = new InstantCommand(() -> {
        switch (currentState) {
            case AUTO:
                this.setTurretOff.schedule(); break;
            case FORWARD:
                this.setTurretAuto.schedule(); break;
            case OFF:
                this.setTurretForward.schedule(); break;
        }
    });
}
