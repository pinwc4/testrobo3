package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SleeveCamera implements VisionProcessor {

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(180, 410);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 25;
    public static int REGION_HEIGHT = 25;
    public Rect selectedRec = new Rect(180, 410, 25, 25);
    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    public ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Get the submat frame, and then sum all the values
        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        // Get the minimum RGB value from every single channel
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        // Change the bounding box color based on the sleeve color
        if (sumColors.val[0] == minColor) {
            position = ParkingPosition.CENTER;
        } else if (sumColors.val[1] == minColor) {
            position = ParkingPosition.RIGHT;
        } else {
            position = ParkingPosition.LEFT;

        }

        // Release and return input
        areaMat.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();

        android.graphics.Rect andSelectedRec = makeGraphicsRect(selectedRec, scaleBmpPxToCanvasPx);
        switch (position) {
            case CENTER:
                selectedPaint.setColor(Color.CYAN);
                selectedPaint.setStyle(Paint.Style.STROKE);
                selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
                canvas.drawRect(andSelectedRec, selectedPaint);
                break;
            case RIGHT:
                selectedPaint.setColor(Color.MAGENTA);
                selectedPaint.setStyle(Paint.Style.STROKE);
                selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
                canvas.drawRect(andSelectedRec, selectedPaint);
                break;
            default:
                selectedPaint.setColor(Color.YELLOW);
                selectedPaint.setStyle(Paint.Style.STROKE);
                selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
                canvas.drawRect(andSelectedRec, selectedPaint);
                break;
        }
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

}
