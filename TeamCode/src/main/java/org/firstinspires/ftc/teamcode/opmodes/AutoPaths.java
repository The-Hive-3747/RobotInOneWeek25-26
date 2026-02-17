package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class AutoPaths {
    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose, lastShootingPose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, lineUpForOpenGate, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park, openGate, toShootFromOpenGate, lineUpForIntake3, intake3, toShootFromIntake3;
    public static double shootAngle, parkAngle, startAngle, intakeAngle, lastShootAngle;
    public static Alliance alliance;

    /**
     * Flips a Pose over the center line.
     *
     * @param Pose, your start pose
     * @return Pose which has been flipped
     */ 

    private static Pose flipOverCenter(Pose pose) {
        if (alliance == Alliance.BLUE) {
            return pose;
        }

        // we subtract the x from 144 to flip the x 
        // y stays the same for this game
        double newPoseX = 144-pose.getX();
        return new Pose(newPoseX, pose.getY());
    }
    
    private static double flipHeading180Degrees(double heading) {
      return Math.toRadians(heading + 180);
   }

    public static void setStartPose(Pose pose) {
      startingPose = pose;
    } 

    public static Alliance getAlliance() {
        return alliance;
    }

    public static PathChain generatePath(Pose pose1, Pose pose2) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(pose1.heading, pose2.heading)
                .build();
    }

    public static PathChain generatePath(Pose pose1, Pose pose2, double heading) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setConstantHeadingInterpolation(heading)
                .build();
    }

    public static PathChain generatePath(Pose pose1, Pose pose2, double heading1, double heading2) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(heading1, heading2)
                .build();
    }


    public static void generatePaths(Follower follower) {
        if (startingPose == null) {
          if (alliance == Alliance.BLUE) {
            startingPose = new Pose(foo);
          } else {
            startingPose = new Pose(foo);
          }
        } 
        
        // DEFINE POSES HERE
        shootingPose = new Pose(foo);


        // GENERATE PATHS HERE

        toShootFromStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startingPose, shootingPose)
                )
                .setLinearHeadingInterpolation(startAngle, shootAngle)
                .build();

    }
}   
