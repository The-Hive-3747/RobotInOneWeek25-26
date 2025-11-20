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
    boolean isRed;
    double goalAngle, goalDiff, goalX, goalY, error;
    ControlSystem turretPID;

    @Override
    public void postInit() {
        turretMotor = new MotorEx("turret");
        //turretMotor.reverse();
        turretMotor.zero();
        error = 0;

    }


    public void update() {
        setGoalAngle();
        ActiveOpMode.telemetry().addData("angle", error);
    }

    public void setGoalAngle() {
        if (currentPose != null) {
            goalAngle = Math.atan2(goalY- currentPose.getY(), goalX-currentPose.getX()); // IN RADS
            //goalAngle = Math.toDegrees(goalAngle); // IN DEGREES
            goalDiff = normalizeAngle(goalAngle + currentPose.getHeading());
            error = Math.toDegrees(normalizeAngle(goalDiff - Math.toRadians(this.getTurretAngle())));
            turretMotor.setPower(0.015*error);
            ActiveOpMode.telemetry().addData("power", 0.01*error);
        }
    }

    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
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
