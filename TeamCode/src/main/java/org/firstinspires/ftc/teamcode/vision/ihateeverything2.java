package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class ihateeverything2 extends NextFTCOpMode {
Mat testImg;
Mat markers;
    @Override
    public void onInit() {
        telemetry.addLine("i hate life");
        testImg = new Mat(240, 320, CvType.CV_8UC3, new Scalar(0, 0, 0));

        Imgproc.circle(testImg, new Point(130, 120), 50, new Scalar(255, 255, 255), -1);
        Imgproc.circle(testImg, new Point(190, 120), 50, new Scalar(255, 255, 255), -1);

        markers = new Mat(240, 320, CvType.CV_32S, new Scalar(0));
        markers.setTo(new Scalar(1));

        Imgproc.circle(markers, new Point(130, 120), 10, new Scalar(2), -1);
        Imgproc.circle(markers, new Point(190, 120), 10, new Scalar(2), -1);

    }

    @Override
    public void onUpdate() {
        try {

            Imgproc.watershed(testImg, markers);
            testImg.release();
            markers.release();
            telemetry.addLine("yay it worked!");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("an exception occured", e.toString());
            telemetry.update();
        }

    }
}
