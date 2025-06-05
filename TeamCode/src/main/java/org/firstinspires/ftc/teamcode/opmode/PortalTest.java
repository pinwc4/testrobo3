package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SleeveCamera;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = ("Portal Test"))
public class PortalTest extends OpMode {

    private VisionPortal visionPortal;

    public int left = 220;
    public int top = 195;
    public PredominantColorProcessor colorSensor;

    @Override
    public void init() {
         colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(left, top, left + 10, top + 10))
                .setSwatches(
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.YELLOW)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(640,480))
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .build();
    }

    @Override
    public void init_loop() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        telemetry.addData("ROTATION: ", result.closestSwatch);
        telemetry.update();
    }

    @Override
    public void loop() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        telemetry.addData("ROTATION: ", result.closestSwatch);
        telemetry.update();
    }

    @Override
    public void stop() {

        visionPortal.stopStreaming();
        visionPortal.close();
    }
}
