
package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.ChoppedIntake;
import org.firstinspires.ftc.teamcode.helpers.GBLight;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.vision.CustomCamera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.vision.opencv.Circle;
import org.firstinspires.ftc.teamcode.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.vision.opencv.ColorRange;
import org.firstinspires.ftc.teamcode.vision.opencv.ImageRegion;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Vision Auto", group = "Concept")
public class ArtifactFetcher extends NextFTCOpMode {
    {
        addComponents(
                drive = new RobotCentricDrive(), // adding robot centric drive component so that the robot can drive
                light = new GBLight()
                //,intake = new ChoppedIntake() //yerassyl: i added new component named intake
        );
    }
    //ChoppedIntake intake;
    RobotCentricDrive drive;
    GBLight light;
    public ColorBlobLocatorProcessor purpleLocator, greenLocator;
    ControlSystem rotController, yController, xController;
    public double Pr = 0.003;
    public double Py = 0.02;
    public double Px = 0.002;
    public double camCenter = 160;
    public double blobRadGoal = 50;
    public double INTAKE_ACTIVATION_RADIUS = 45;

    @Override
    public void onInit() {
         purpleLocator = new ColorBlobLocatorProcessor.Builder() // creating a new PURPLE color blob locator. this is an sdk example!
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame()) // use the entire camera frame
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();
        greenLocator = new ColorBlobLocatorProcessor.Builder() // creating a new GREEN color blob locator. this is an sdk example!
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame()) // use the entire camera frame
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        CustomCamera portal = new CustomCamera.Builder() // Building a new vision portal
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // setting the camera
                .addProcessors(greenLocator, purpleLocator) // adding the two processors i just made
                .setCameraResolution(new Size(320, 240)) // setting the resolution of the camera. lower resolution = faster looptimes
                .setStreamFormat(CustomCamera.StreamFormat.MJPEG) // have to use MJPEG for the OV camera
                .enableLiveView(true)
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging. (normally its 250ms)

        // these PIDs are using NextFTC
        rotController = ControlSystem.builder() // creating a new PID to control our rotation
                .angular(AngleType.DEGREES, // need to use angular so the robot properly turns instead of going 360
                    feedback -> feedback.posPid(Pr, 0, 0) // position pid
                )
                .build();
        yController = ControlSystem.builder() // this PID controls the robot driving forward
                .posPid(Py, 0,0.01) // position PID
                .build();
        xController = ControlSystem.builder() // this PID controls the robot strafing
                .posPid(Px, 0,0) // position PID
                .build();

        // i dont want the controllers to do anything for now so they can chill :)
        rotController.setGoal(new KineticState(0.0));
        xController.setGoal(new KineticState(0.0));
        yController.setGoal(new KineticState(0.0));
    }
    @Override
    public void onUpdate() {
            // creates a new ArrayList of Blobs. I want to use both purple blobs and green blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
            blobs.addAll(greenLocator.getBlobs());

            // you can filter blobs to remove unwanted data & distractions. these are from the SDK
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, // contour area is the size the blob takes
                    100, 20000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, // circularity is how circular the blob is
                    0.5, 1, blobs);     // filter out non-circular blobs.


            // I want to sort the blobs so that the robot focuses on the largest blob first.
            // remember, contour area is the size of the blob
            ColorBlobLocatorProcessor.Util.sortByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,  SortOrder.DESCENDING, blobs
            );

            // telemtry stuff from the sdk
            telemetry.addLine("Circularity Radius Center");
            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                Circle circleFit = b.getCircle();
                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
            }

            // this is my code for the PIDs!
            if (!blobs.isEmpty()) { // i only want to be moving if there are blobs that i can see.
                // goals are where the PID is going to try to get the value to.
                // the point of the robot strafing/rotating is to get the robot in the center of the camera (in the x direction)
                rotController.setGoal(new KineticState(camCenter)); // kinetic states are NextFTC's way of storing information
                xController.setGoal(new KineticState(camCenter));
                // the point of the robot moving forward is to get the blob to a specific size. this uses the blob's radius!
                yController.setGoal(new KineticState(blobRadGoal));

                // drive is my robot centric drive. i feed it my y, x, and rot, and it moves the robot!
                // it works the same way as moving ur joystick
                drive.update(
                        // out of my list of sorted blobs, get the first one (in an array that's 0), get the circle, and get the lateral x.
                        // it's going to try to get the x to be equal to 160 (the center of the camera)
                        rotController.calculate(new KineticState(blobs.get(0).getCircle().getX())),
                        // same thing, but it's trying to change the radius
                        yController.calculate(new KineticState(blobs.get(0).getCircle().getRadius())),
                        xController.calculate(new KineticState(blobs.get(0).getCircle().getX()))
                );
               /* if (blobs.get(0).getCircle().getRadius() > INTAKE_ACTIVATION_RADIUS && intake.getStoredArtifact() == ChoppedIntake.ArtifactColor.NONE) {
                    intake.run(0.7);
                } else {
                    intake.stop();
                } */
            } else {
                // if there are no blobs, my pids shouldnt be trying to do anything and my robot shouldnt drive on its own
                rotController.setGoal(new KineticState(0));
                yController.setGoal(new KineticState(0));
                xController.setGoal(new KineticState(0));
                drive.update(0.4, 0.0, 0.0);
                //intake.stop();
            }
            /*if (gamepad1.a && intake.getStoredArtifact() != ChoppedIntake.ArtifactColor.NONE) {
                intake.reset();
                telemetry.addData("Intake is Reset","yes");
            }*/
            //telemetry.addData("Artifact", intake.getStoredArtifact());
            //telemetry.addData("Color Sensor val", String.valueOf(intake.alpha), intake.red, intake.blue, intake.green);
            telemetry.update();
        }
    }
