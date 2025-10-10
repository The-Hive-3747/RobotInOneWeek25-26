package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

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

import java.util.ArrayList;
import java.util.List;

public class WatershedProc implements VisionProcessor {

    public static final Scalar LOWER_GREEN = new Scalar(40, 100, 100);
    public static final Scalar UPPER_GREEN = new Scalar(80, 255, 255);

    private static final double SURE_FG_THRESHOLD_RATIO = 0.5;

    private Mat hsvMat, mask, opening, sureBg, distTransform, sureFg, unknown, markers;
    private List<MatOfPoint> finalContours = new ArrayList<>();
    private long lastProcTimeMs = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        hsvMat = new Mat();
        mask = new Mat();
        opening = new Mat();
        sureBg = new Mat();
        distTransform = new Mat();
        sureFg = new Mat();
        unknown = new Mat();
        markers = new Mat();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long startTime = System.nanoTime();
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, LOWER_GREEN, UPPER_GREEN, mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1),2);
        Imgproc.dilate(opening, sureBg, kernel, new Point(-1,-1),3);
        Imgproc.distanceTransform(opening,distTransform,Imgproc.DIST_L2,5);
        Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
        Imgproc.threshold(distTransform, sureFg, SURE_FG_THRESHOLD_RATIO, 1, Imgproc.THRESH_BINARY);
        sureFg.convertTo(sureFg, CvType.CV_8U);
        Core.subtract(sureBg, sureFg, unknown);
        Imgproc.connectedComponents(sureFg, markers);
        Core.add(markers, new Scalar(1), markers);
        markers.setTo(new Scalar(0), unknown);

        Imgproc.watershed(frame, markers);

        finalContours.clear();

        int numObjects = (int) Core.minMaxLoc(markers).maxVal;

        for (int i = 2; i<=numObjects; i++) {
            Mat componentMask = new Mat(markers.size(), CvType.CV_8UC1);
            try {
                Core.compare(markers, new Scalar(i), componentMask, Core.CMP_EQ);
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(componentMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                finalContours.addAll(contours);
            } finally {
                componentMask.release();
            }
        }

        long endTime = System.nanoTime();
        lastProcTimeMs = (endTime - startTime) / 1_000_000;

        return finalContours;
    }

    @Override public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint contourPaint = new Paint();
        contourPaint.setColor(Color.YELLOW);
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setStrokeWidth(5 * scaleCanvasDensity);
        Paint textPaint = new Paint();
        textPaint.setColor(Color.CYAN);
        textPaint.setTextSize(40 * scaleCanvasDensity); // Draw all the final contours
        for (MatOfPoint contour : finalContours) {
            // Scale and draw the contour
            Point[] points = contour.toArray();
            for (int i = 0; i < points.length; i++) {
                points[i].x *= scaleBmpPxToCanvasPx;
                points[i].y *= scaleBmpPxToCanvasPx;
            }
            for (int i = 0; i < points.length; i++) {
                canvas.drawLine((float)points[i].x, (float)points[i].y, (float)points[(i + 1) % points.length].x, (float)points[(i + 1) % points.length].y, contourPaint);
            }
        } // Display the number of objects found
        if (userContext != null) {
            canvas.drawText(String.format("%d objects found", (int) userContext), 50, 100, textPaint);
        }

        canvas.drawText(String.format("Time: %d ms", lastProcTimeMs), 50, 150, textPaint);
    }

}
