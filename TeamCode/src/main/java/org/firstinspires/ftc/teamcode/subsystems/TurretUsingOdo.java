package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretUsingOdo implements Component {

    MotorEx turretMotor;
    Pose currentPose;
    boolean isRed, allowTurret;
    double goalAngle, goalDiff, goalX, goalY, turretPower, turretGoal;
    ControlSystem turretPID;

    @Override
    public void postInit() {
        turretMotor = new MotorEx("turret");

        turretMotor.reverse();
        turretMotor.zero();
        turretMotor.atPosition(0);
        ActiveOpMode.telemetry().addData("pos", turretMotor.getCurrentPosition());
        ActiveOpMode.telemetry().update();
        turretPID = ControlSystem.builder()
                .posPid(0.02, 0, 0.001)
                .build();
        turretGoal = 0;

    }


    public void update() {
        turretMotor.zero();
        if (allowTurret) {
            //setGoalAngle();
        } else {
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
            //turretMotor.setPower(turretPower);
        }
        ActiveOpMode.telemetry().addData("goal angle", Math.toDegrees(goalAngle));
        ActiveOpMode.telemetry().addData("goal diff", Math.toDegrees(goalDiff));
        ActiveOpMode.telemetry().addData("posesesese", currentPose);
        ActiveOpMode.telemetry().addData("turret angle", this.getTurretAngle());
        ActiveOpMode.telemetry().addData("turret goal", turretGoal);
    }

    public void setGoalAngle() {
        if (currentPose != null) {
            goalAngle = -Math.atan2((goalY - this.currentPose.getY()), (goalX - this.currentPose.getX())); // IN RADS
            //goalAngle = Math.toDegrees(goalAngle); // IN DEGREES
            turretGoal = normalizeAngle(goalAngle + currentPose.getHeading());
            turretPID.setGoal(new KineticState(this.putInTurretLimits(Math.toDegrees(turretGoal))));
            // error = Math.toDegrees(normalizeAngle(goalDiff - Math.toRadians(this.getTurretAngle())));
            //turretMotor.setPower(0.02*error);
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
            turretMotor.setPower(turretPower);
        }
    }

    public double putInTurretLimits(double goal) {
        if (goal > 80 || goal <-200) { //insanely horribly code
            if (goal > 80) {
                goal = 80;
            } else {
                goal = -200;
            }
        }
        return goal;
    }

    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    public void setTurretAngle(double goal) {
        allowTurret = false;
        turretPID.setGoal(new KineticState(goal));
    }
    public void allowTurret() {
        allowTurret = true;
    }

    public double getTurretAngle() {
        return (turretMotor.getCurrentPosition()*90)/212;
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

    public void setAlliance(boolean alliance) {
        this.isRed = alliance;
        if (alliance) {
            goalX = 144;
            goalY = 144;
        } else {
            goalX = 0;
            goalY = 144;
        }
    }
}
