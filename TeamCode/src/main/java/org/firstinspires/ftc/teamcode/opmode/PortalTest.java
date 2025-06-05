package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SleeveCamera;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = ("Camp Bot"))
public class PortalTest extends OpMode {
    private String webcamName = "webcam";
    private VisionPortal visionPortal;
    public SleeveCamera cameraSubsystem;

    @Override
    public void init() {
        cameraSubsystem = new SleeveCamera();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(cameraSubsystem)
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        visionPortal.stopStreaming();
    }
}
