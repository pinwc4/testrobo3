package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public enum ControlType {
        FIELDCENTRIC,
        ROBOTCENTRIC
    }
    public SampleMecanumDrive drive;
    public IMU imu;
    public IntegratingGyroscope navxgyro;
    public NavxMicroNavigationSensor navxMicro;
    public ElapsedTime timer = new ElapsedTime();
    public ControlType controls;

    public Robot(HardwareMap hardwareMap) {
        //Define hardware map items first
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        navxgyro = (IntegratingGyroscope)navxMicro;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controls = ControlType.FIELDCENTRIC;



    }
}
