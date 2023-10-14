package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

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
    public Servo clawServo;
    public Claw clawSubsystem;
    public DistanceSensor rightDistanceSensor;
    public HuskyLens huskyLens;
    private Thread i2cThread;
    private final Object i2cLock = new Object();
    public Boolean stopThread = false;
    public double rightDistance;
    public HuskyLens.Block[] huskyBlocks;
    public Orientation navxAngles;
    public double navxHeading;
    public double imuHeading;



    public Robot(HardwareMap hardwareMap) {
        //Define hardware map items first
        imu = hardwareMap.get(IMU.class, "imu");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        clawServo = hardwareMap.get(Servo.class, "claw");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        navxgyro = (IntegratingGyroscope)navxMicro;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controls = ControlType.FIELDCENTRIC;

        clawSubsystem = new Claw(clawServo);

        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistance");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(clawSubsystem);
    }

    public void startI2CThread(){
        i2cThread = new Thread(() -> {
            double t_rightDistance = 0;
            HuskyLens.Block[] t_blocks;
            Orientation t_navxAngles;
            double t_imuHeading;
            while (!stopThread) {
                t_rightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
                t_blocks = huskyLens.blocks();
                t_navxAngles = navxgyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                t_imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                synchronized (i2cLock){
                    rightDistance = t_rightDistance;
                    huskyBlocks = t_blocks;
                    navxHeading = t_navxAngles.firstAngle;
                    imuHeading = t_imuHeading;
                }
            }
        });
        i2cThread.start();
    }

}
