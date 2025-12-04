package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class TurretUsingOdo implements Component {
    DcMotorEx turret;
    Pose currentPose;
    boolean isRed, allowTurret;
    double goalAngle, goalDiff, goalX, goalY, turretPower, turretGoal;
    ControlSystem turretPID;

    @Override
    public void postInit() {
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPID = ControlSystem.builder()
                .posPid(0.02, 0, 0.001)
                .build();
        turretGoal = 0;
    }

    @Override
    public void postStartButtonPressed() {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (allowTurret) {
            getGoalAngle();
        } else {
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
        }
        if (turretPower > 0.5) {
            turretPower = 0.5;
        }
        turret.setPower(turretPower);
        ActiveOpMode.telemetry().addData("goal angle", Math.toDegrees(goalAngle));
        ActiveOpMode.telemetry().addData("goal diff", Math.toDegrees(goalDiff));
        ActiveOpMode.telemetry().addData("posesesese", currentPose);
        ActiveOpMode.telemetry().addData("turret angle", this.getTurretAngle());
        ActiveOpMode.telemetry().addData("turret goal", turretGoal);
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
        if (goal > 70 || goal <-200) { //insanely horribly code
            if (goal > 70) {
                goal = 70;
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
        return (turret.getCurrentPosition()*90)/212;
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
