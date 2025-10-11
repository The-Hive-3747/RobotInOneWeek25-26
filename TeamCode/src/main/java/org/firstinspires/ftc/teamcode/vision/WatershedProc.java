package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.helpers.DetectedArtifact;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class WatershedProc extends OpenCvPipeline {

    public enum DebugView {
        COLOR_MASK,
        NOISE_REDUCTION,
        ARTIFACT_CORES,
        INPUT_WATERSHED,
        WATERSHED,
        FINAL
    }

    public static DebugView VIEW_MODE = DebugView.FINAL;

    public static final Scalar LOWER_GREEN = new Scalar(50, 50, 50);
    public static final Scalar UPPER_GREEN = new Scalar(100, 255, 255);

    private volatile int numObjectsFound = 0;
    private List<MatOfPoint> finalContours = new ArrayList<>();

    private static final double SEPARATION_THRESHOLD = 0.6;
    private static final int MORPH_OPEN_ITERATIONS = 5;

    private Mat hsvMat = new Mat();
    private Mat mask  = new Mat();
    private Mat opening = new Mat();
    private Mat distTransform = new Mat();
    private Mat sureFg = new Mat();
    private Mat markers = new Mat();
    private Mat hierarchy = new Mat();
    private Mat inputForWatershed = new Mat();
    private Mat gray = new Mat();
    private Mat unknown = new Mat();
    private Mat sureBg = new Mat();
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
    public volatile List<DetectedArtifact> detectedArtifacts = new ArrayList<>();


    public boolean changeViewMode() {
        switch (VIEW_MODE) {
            case COLOR_MASK:
                VIEW_MODE = DebugView.NOISE_REDUCTION;
                return true;
            case NOISE_REDUCTION:
                VIEW_MODE = DebugView.INPUT_WATERSHED;
                return true;
            case INPUT_WATERSHED:
                VIEW_MODE = DebugView.WATERSHED;
                return true;
            case WATERSHED:
                VIEW_MODE = DebugView.ARTIFACT_CORES;
                return true;
            case FINAL:
                VIEW_MODE = DebugView.COLOR_MASK;
                return true;
            case ARTIFACT_CORES:
            default:
                VIEW_MODE = DebugView.FINAL;
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

        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1),MORPH_OPEN_ITERATIONS);

        Imgproc.distanceTransform(opening,distTransform,Imgproc.DIST_L2,5);
        Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
        Imgproc.threshold(distTransform, sureFg, SEPARATION_THRESHOLD, 1, Imgproc.THRESH_BINARY);
        sureFg.convertTo(sureFg, CvType.CV_8U);
        Imgproc.connectedComponents(sureFg, markers);

        Imgproc.dilate(opening, sureBg, kernel, new Point(-1,-1), 3);
        Core.subtract(sureBg, sureFg, unknown);
        Core.add(markers, new Scalar(1), markers);
        markers.setTo(new Scalar(0), unknown);
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(gray, inputForWatershed, Imgproc.COLOR_GRAY2RGB);
        Imgproc.watershed(inputForWatershed, markers);

        List<DetectedArtifact> currentArtifacts = new ArrayList<>();
        int numObjects = (int) Core.minMaxLoc(markers).maxVal;

        for (int i = 2; i <= numObjects; i++) {
            Mat componentMask = new Mat(markers.size(), CvType.CV_8UC1);
            Core.compare(markers, new Scalar(i), componentMask, Core.CMP_EQ);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(componentMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint contour = contours.get(0);
                double area = Imgproc.contourArea(contour);
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                currentArtifacts.add(new DetectedArtifact(rotatedRect.center, area, rotatedRect.size.width, rotatedRect.size.height));

                Point[] vertices = new Point[4];
                rotatedRect.points(vertices);
                for (int j = 0; j<4; j++) {
                    Imgproc.line(input, vertices[j], vertices[(j+1)%4], new Scalar(0,255,0), 2);
                }
                Imgproc.putText(input, String.format("W:%.0f H:%.0f", rotatedRect.size.width, rotatedRect.size.height), rotatedRect.center, Imgproc.FONT_ITALIC, 0.5, new Scalar(255,255,255), 1);
            }
            componentMask.release();
        }

        this.detectedArtifacts = currentArtifacts;
        Imgproc.putText(input, "Found: " + detectedArtifacts.size(), new Point(10,30), Imgproc.FONT_ITALIC, 0.7, new Scalar(255,255,255), 2);



        switch (VIEW_MODE) {
            case COLOR_MASK: return mask;
            case NOISE_REDUCTION: return opening;
            case ARTIFACT_CORES:
                sureFg.convertTo(sureFg, CvType.CV_8U, 255);
                return sureFg;
            case INPUT_WATERSHED:
                return inputForWatershed;
            case WATERSHED:
                return markers;
            case FINAL:
            default:
                return input;
        }
    }

    public int getNumObjectsFound() {
        return numObjectsFound;
    }
}
