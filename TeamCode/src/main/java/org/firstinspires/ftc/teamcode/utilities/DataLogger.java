package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

public class DataLogger implements Component{

    DcMotorEx flywheelBottom, intake;
    CRServo hood;
    Pose currentPose = OpModeTransfer.currentPose;
    double botDistance;
    Alliance alliance;
    double goalX;
    double goalY;


    @Override
    public void postInit() {
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        intake = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");
    }

    public void update() {
        ActiveOpMode.telemetry().addData("Bot Position",this.currentPose); //need to put follower in current pose in update of opmode
        ActiveOpMode.telemetry().addData("Distance to Goal",this.getBotDistance());
        ActiveOpMode.telemetry().addData("Flywheel vel",flywheelBottom.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel goal vel","uhh");
        ActiveOpMode.telemetry().addData("hood pos", -intake.getCurrentPosition()); //hood encoder is on intake
        ActiveOpMode.telemetry().addData("time shooting","");
        ActiveOpMode.telemetry().update();

    }

    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 129; //this is the point of the middle of the front panel of the goal
            goalY = 129;
        } else {
            goalX = 15;
            goalY = 129;
        }
    }
    public double getBotDistance() {
        //this is math for the distance from bot to goal using hypotenuse of x and y
        botDistance = Math.sqrt(Math.pow(goalX - currentPose.getX(), 2) + Math.pow(goalY - currentPose.getY(), 2));
        return botDistance;
    }
    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }


}
