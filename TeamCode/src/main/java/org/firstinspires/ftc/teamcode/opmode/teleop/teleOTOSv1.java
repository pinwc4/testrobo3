package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "OTOSv1")
public class teleOTOSv1 extends OpMode {
    SampleMecanumDrive drive;
    IMU revIMU;
    IntegratingGyroscope navxgyro;
    NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer;
    SparkFunOTOS myOtos;
    SparkFunOTOS.Pose2D posOtos;
    Follower follower;
    Pose2d poseEstimate;
    double starttime;
    double elapsedtime;
    Orientation navxAngles;
    AngularVelocity revAngularVelocity;
    YawPitchRollAngles revOrientation;

    @Override
    public void init() {
        //Configure roadrunner to read from dead wheels
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init the built in IMU
        revIMU = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        revIMU.initialize(parameters);

        //External Navx gyro
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        navxgyro = (IntegratingGyroscope)navxMicro;

        timer = new ElapsedTime();

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        //Set unit types
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);
        //Configure sensor offsets
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        //Must tune scalars, push robot 100 inches and check reading to adjust
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        //Takes about half a second
        myOtos.calibrateImu();
        myOtos.resetTracking();
        //Specify field location
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        //setup pedro follower
        follower = new Follower(hardwareMap);
        follower.setPose(new Pose(0,0, 0));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
    }

    @Override
    public void loop() {
        starttime = timer.milliseconds();

        poseEstimate = drive.getPoseEstimate();
        posOtos = myOtos.getPosition();

        navxAngles = navxgyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        revAngularVelocity = revIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        revOrientation = revIMU.getRobotYawPitchRollAngles();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        elapsedtime = timer.milliseconds() - starttime;

        telemetry.addData("loop ms", elapsedtime);
        telemetry.addData("OTOS X", String.format("%.5g",posOtos.x));
        telemetry.addData("OTOS Y", String.format("%.5g",posOtos.y));
        telemetry.addData("OTOS heading", String.format("%.5g",Math.toDegrees(posOtos.h)));
        telemetry.addData("GB X", String.format("%.5g",poseEstimate.getX()));
        telemetry.addData("GB Y", String.format("%.5g",poseEstimate.getY()));
        telemetry.addData("GB heading", String.format("%.5g",Math.toDegrees(poseEstimate.getHeading())));
        telemetry.addData("navx heading", String.format("%.5g",navxAngles.firstAngle));
        telemetry.addData("imu heading", String.format("%.5g",revOrientation.getYaw(AngleUnit.DEGREES)));
    }

}
