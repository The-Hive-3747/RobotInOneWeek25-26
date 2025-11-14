package org.firstinspires.ftc.teamcode.pathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.65)//V2 robot mass
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .lateralZeroPowerAcceleration(-37.32)//-87.57
            .forwardZeroPowerAcceleration(-35.8)//-52.34
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.005, 0))//(0.03, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.00, 0))//(0.1, 0, 0.01, 0))//just a guess
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0,0.000,0.6,0.015))//(0.09, 0, 0.0001, 0.0, 0.00))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.09, 0, 0.000005, 0.6, 0.01))//(0.001, 0, 0.00001, 0, 0.01))//just a guess
            .centripetalScaling(0.0005)//(0.001)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0, 0.01))//(1, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.01, -0.0008))//(0.1, 0, 0.01, 0))//just a guess
            .drivePIDFSwitch(5);//5



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(64.03)
            .yVelocity(50.0);//(24.53);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.4)//(0)
            .strafePodX(0)//(-5.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.85, 0.5, 0.1, 0.009, 50, 1.25, 10, 1);//(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}