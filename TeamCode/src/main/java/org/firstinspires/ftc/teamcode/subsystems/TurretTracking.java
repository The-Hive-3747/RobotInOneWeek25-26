package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretTracking implements Component{
    private Limelight3A limelight;
    double x;
    double y;
    double z;
    double center;
    double centerP;
    double horizDistance;
    private MotorEx turretMotor;

    @Override
    public void postInit() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        turretMotor = new MotorEx("turret").brakeMode();
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public void update() {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
            Pose3D botPose = fiducial.getRobotPoseTargetSpace();
            centerP = fiducial.getTargetXPixels();
            center = fiducial.getTargetXDegrees();
            y = botPose.getPosition().y;
            z = botPose.getPosition().z;
            x = botPose.getPosition().x;

            horizDistance = Math.sqrt(x*x + y*y);

            if (center>5) {
                turretMotor.setPower(0.3);
            }else if (center<-5) {
                turretMotor.setPower(-0.3);
            }else {
                turretMotor.setPower(0);
            }

            ActiveOpMode.telemetry().addData("horizontal distance", horizDistance);
            ActiveOpMode.telemetry().addData("april tag", fiducial.getFiducialId());
            ActiveOpMode.telemetry().addData("z distance", z);
            ActiveOpMode.telemetry().addData("x distance", x);
            ActiveOpMode.telemetry().addData("y distance", y);
            ActiveOpMode.telemetry().addData("center distance", center);
        }

    }

    Command holdTurret = new LambdaCommand()
            .setStart(() -> {
                turretMotor.setPower(0);
            })
            .setIsDone(() -> true);
    }
