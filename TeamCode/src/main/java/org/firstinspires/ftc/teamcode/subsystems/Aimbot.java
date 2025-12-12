package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;


public class Aimbot implements Component{
    GoBildaPinpointDriver odo;
    
    Pose currentPose;
    double velocity;
    double percentage;
    double hoodPos;
    double botDistance;
    double goalX;
    double goalY;
    Alliance alliance;
    AimbotValues currentAimValues;

    public void preInit() {


    }
    public void postInit() {
        AimbotValues currentAimValues = getAimValues(botDistance);
    }
    public void update() {
        botDistance = this.getBotDistance();
        currentAimValues = this.getAimValues(botDistance);
        ActiveOpMode.telemetry().addData("CURRENT AIM VALUES", currentAimValues.velocity);
        ActiveOpMode.telemetry().addData("CURRENT AIM VALUES", currentAimValues.hoodPos);
    }
    
    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }
    public Pose getCurrentPose() {
        return this.currentPose;
    }
    AimbotValues[] aimbotValues = {
            new AimbotValues(16.5, 850, 0),
            new AimbotValues(21.5, 950, 0),
            new AimbotValues(26.5, 950, 0),
            new AimbotValues(31.5, 950, 727),
            new AimbotValues(36.5, 950, 1003),
            new AimbotValues(41.5, 1000, 1243),
            new AimbotValues(46.5, 1050, 1237),
            new AimbotValues(51.5, 1000, 1250),
            new AimbotValues(56.5, 1050, 1243),
            new AimbotValues(61.5, 1050,1264),
            new AimbotValues(69.5, 1200, 1762),
            new AimbotValues(76.5, 1250, 1903),
            new AimbotValues(111.5, 1350, 1250),
            new AimbotValues(116.5, 1350, 1750),
            new AimbotValues(121.5, 1400, 1500),
            new AimbotValues(126.5, 1450, 2000),
            new AimbotValues(131.5, 1450, 2000),
    };
    public AimbotValues getAimValues(double distance){
        AimbotValues next;
        AimbotValues last;
        last = null;

        percentage = 0;
        velocity = aimbotValues[0].velocity;
        hoodPos = aimbotValues[0].hoodPos;

        for(AimbotValues value : aimbotValues) {
            next = value;
            if (next != null && last != null && distance >= last.distance && distance < next.distance) {
                percentage = (distance - last.distance)/(next.distance-last.distance);
                velocity = (next.velocity - last.velocity) * percentage + last.velocity;
                hoodPos = (next.hoodPos - last.hoodPos) * percentage + last.hoodPos;
            }else if(next != null && last != null && distance > last.distance && distance >= next.distance){
                percentage = 0;
                velocity = next.velocity;
                hoodPos = next.hoodPos;
            }
            last = next;
        }
        return new AimbotValues(distance, velocity, hoodPos);
    }
    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 129;
            goalY = 129;
        } else {
            goalX = 15;
            goalY = 129;
        }
    }
    public double getBotDistance() {
        botDistance = Math.sqrt(Math.pow(goalX - currentPose.getX(), 2) + Math.pow(goalY - currentPose.getY(), 2));
        return botDistance;
    }

    public AimbotValues getAimbotValues() {
        return currentAimValues;
    }
}

