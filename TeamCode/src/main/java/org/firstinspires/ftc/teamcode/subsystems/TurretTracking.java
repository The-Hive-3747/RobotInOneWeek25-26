package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilder;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretTracking implements Component{
    private Limelight3A limelight;
    static double x, y, z, center, centerP, horizDistance, correct;
    private MotorEx turretMotor;
    ControlSystem turretPID;
    int initTag;
    int targetTagID;

    @Override
    public void postInit() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        turretMotor = new MotorEx("turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.zero();
        limelight.pipelineSwitch(0);
        limelight.start();
        center = 0;
        targetTagID=24;
        turretPID = new ControlSystemBuilder()
                .posPid(0.1)
                .build();
        turretPID.setGoal(
                new KineticState(0)
        );
    }

    public void setAllianceColor(boolean isRed) {
        if (isRed) { 
            targetTagID = 24; 
        } else {
            targetTagID = 20;
        }
    }

    public void update() {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if (result.getFiducialResults().get(i).getFudicialId() == targetTagID) {
                    LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                    Pose3D botPose = fiducial.getRobotPoseTargetSpace();
                    centerP = fiducial.getTargetXPixels();
                    center = fiducial.getTargetXDegrees();
                    y = botPose.getPosition().y;
                    z = botPose.getPosition().z;
                    x = botPose.getPosition().x;
                    horizDistance = Math.sqrt(x*x + y*y);
                    break;
                }
            }
            
            
            correct = turretPID.calculate(
                    new KineticState(center)
            );
            if (Math.abs(correct) < 1) {
                turretMotor.setPower(correct);
            } else {
                turretMotor.setPower(0);
            }


            ActiveOpMode.telemetry().addData("horizontal distance", horizDistance);
            ActiveOpMode.telemetry().addData("april tag", fiducial.getFiducialId());
            ActiveOpMode.telemetry().addData("z distance", z);
            ActiveOpMode.telemetry().addData("x distance", x);
            ActiveOpMode.telemetry().addData("y distance", y);
            ActiveOpMode.telemetry().addData("center distance", center);
            ActiveOpMode.telemetry().addData("AprilTag", targetTagID);

        }

    }



    public Command holdTurret = new LambdaCommand()
            .setStart(() -> {

            })
            .setIsDone(() -> true);
    }
