package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Debug;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class WatershedProc extends OpenCvPipeline {

    public enum DebugView {
        RAW_CAMERA_INPUT,
        COLOR_MASK,
        NOISE_REDUCTION_OPENING,
        SURE_FOREGROUND,
        FINAL_WITH_CONTOURS
    }

    public static DebugView VIEW_MODE = DebugView.FINAL_WITH_CONTOURS;

    public static final Scalar LOWER_GREEN = new Scalar(80, 70, 70);
    public static final Scalar UPPER_GREEN = new Scalar(100, 255, 255);

    private volatile int numObjectsFound = 0;
    private List<MatOfPoint> finalContours = new ArrayList<>();

    private static final double SURE_FG_THRESHOLD_RATIO = 0;
    private static final int MORPH_OPEN_ITERATIONS = 2;

    private Mat hsvMat = new Mat();
    private Mat mask  = new Mat();
    private Mat opening = new Mat();
    private Mat sureBg = new Mat();
    private Mat distTransform = new Mat();
    private Mat sureFg = new Mat();
    private Mat unknown = new Mat();
    private Mat markers = new Mat();
    private Mat gray = new Mat();
    private Mat inputForWatershed = new Mat();
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));


    public boolean changeViewMode() {
        switch (VIEW_MODE) {
            case RAW_CAMERA_INPUT:
                VIEW_MODE = DebugView.COLOR_MASK;
                return true;
            case COLOR_MASK:
                VIEW_MODE = DebugView.NOISE_REDUCTION_OPENING;
                return true;
            case NOISE_REDUCTION_OPENING:
                VIEW_MODE = DebugView.SURE_FOREGROUND;
                return true;
            case SURE_FOREGROUND:
                VIEW_MODE = DebugView.FINAL_WITH_CONTOURS;
                return true;
            case FINAL_WITH_CONTOURS:
            default:
                VIEW_MODE = DebugView.RAW_CAMERA_INPUT;
                return true;
        }
    }

    public DebugView getViewMode() {
        return VIEW_MODE;
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, LOWER_GREEN, UPPER_GREEN, mask);
        //Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1),MORPH_OPEN_ITERATIONS);
        //Imgproc.dilate(opening, sureBg, kernel, new Point(-1,-1),3);
        Imgproc.distanceTransform(opening,distTransform,Imgproc.DIST_L2,5);
        Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
        Imgproc.threshold(distTransform, sureFg, SURE_FG_THRESHOLD_RATIO, 1, Imgproc.THRESH_BINARY);
        sureFg.convertTo(sureFg, CvType.CV_8U);
        //Core.subtract(sureBg, sureFg, unknown);
        Imgproc.connectedComponents(sureFg, markers);

        numObjectsFound = (int) Core.minMaxLoc(markers).maxVal;



        Imgproc.dilate(opening, sureBg, kernel, new Point(-1,-1),3);
        Core.subtract(sureBg, sureFg, unknown);
        Core.add(markers, new Scalar(1), markers);
        markers.setTo(new Scalar(0), unknown);

        //Imgproc.watershed(input, markers);

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(gray, inputForWatershed, Imgproc.COLOR_GRAY2RGB);
        Imgproc.watershed(inputForWatershed, markers);


        finalContours.clear();
        int numObjects = (int) Core.minMaxLoc(markers).maxVal;

        for (int i = 2; i<=numObjects; i++) {
            Mat componentMask = new Mat(markers.size(), CvType.CV_8UC1);
            Core.compare(markers, new Scalar(i), componentMask, Core.CMP_EQ);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(componentMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            finalContours.addAll(contours);
            componentMask.release();
        }

        Imgproc.drawContours(input, finalContours, -1, new Scalar(255,255,0), 2);

        switch (VIEW_MODE) {
            case RAW_CAMERA_INPUT: return input;
            case COLOR_MASK: return mask;
            case NOISE_REDUCTION_OPENING: return opening;
            case SURE_FOREGROUND:
                sureFg.convertTo(sureFg, CvType.CV_8U, 255);
                return sureFg;
            case FINAL_WITH_CONTOURS:
            default:
                return input;
        }
    }

    public int getNumObjectsFound() {
        return numObjectsFound;
    }



}
