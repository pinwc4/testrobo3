package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp(name="Switchable Opmode")
public class telopboth  extends OpMode {
    private Robot robot;
    private GamepadEx driverGamepad;
    private Pose2d drivePowers;
    private ToggleButtonReader lockHeading;
    private PIDFController headingController;
    private double currentHeading;
    private double turnAngle;

    //This method runs once when the init button is pressed on the driver hub
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);
        drivePowers = new Pose2d();
        lockHeading = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.X);
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        turnAngle = 0;
    }

    //After init is complete this method runs repeatably after init until the play button is pressed
    /*
    @Override
    public void init_loop() {

    }
    */

    //This method runs once after the start button is pressed
    @Override
    public void start() {
        robot.imu.resetYaw();
        robot.timer.reset();
    }

    //After start is complete this method runs repeatably after the start button is pressed
    @Override
    public void loop() {
        double starttime = robot.timer.milliseconds();
        driverGamepad.readButtons();

        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        //currentHeading = poseEstimate.getHeading();
        currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double gamely = driverGamepad.getLeftY();
        double gamelx = driverGamepad.getLeftX();
        double gamerx = driverGamepad.getRightX();

        //Check to see if we should change control type
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            if (robot.controls == Robot.ControlType.FIELDCENTRIC) {
                robot.controls = Robot.ControlType.ROBOTCENTRIC;
            } else {
                robot.controls = Robot.ControlType.FIELDCENTRIC;
            }
        }

        if (robot.controls == Robot.ControlType.FIELDCENTRIC) {
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    (gamely * Math.abs(gamely)),
                    -(gamelx * Math.abs(gamelx))
            ).rotated(-currentHeading);

            //Check to see if we are locking our heading
            if (lockHeading.getState()) {
                headingController.setTargetPosition(Math.toRadians(90.0));
                double headingInput = (headingController.update(currentHeading) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH;
                drivePowers = new Pose2d(input, headingInput);
            } else {
                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drivePowers = new Pose2d(input.getX(), input.getY(), -(gamerx * Math.abs(gamerx)));
            }

        } else {
            //Robot Centric driving
            drivePowers = new Pose2d((gamely * Math.abs(gamely)), -(gamelx * Math.abs(gamelx)), -(gamerx * Math.abs(gamerx)));

        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || driverGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (currentHeading >= -180.0 && currentHeading < 90.0){
                turnAngle = currentHeading + 90;
            } else {
                turnAngle = currentHeading - 270;
            }
            robot.drive.turn(Math.toRadians(turnAngle));
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)|| driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (currentHeading >= -45.0 && currentHeading <= 180.0){
                turnAngle = currentHeading - 135.0;
            } else {
                turnAngle = currentHeading + 225;
            }
            robot.drive.turnAsync(Math.toRadians(turnAngle));
        }

        robot.drive.setWeightedDrivePower(drivePowers);
        robot.drive.update();

        Orientation angles = robot.navxgyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        double elapsedtime = robot.timer.milliseconds() - starttime;
        // Print pose to telemetry
        telemetry.addData("loop ms", elapsedtime);
        telemetry.addData("mode", robot.controls);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("navx heading", angles.firstAngle);
        telemetry.addData("imu heading", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("locked heading", lockHeading.getState());
        telemetry.update();
        lockHeading.readValue();
    }

    //This runs when the stop button is pressed on the driver hub
    /*
    @Override
    public void stop() {

    }
    */

}
