package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DataStorage;

@TeleOp(name="Alt Heading Opmode")
public class telopAltHeading extends OpMode {

    public Robot robot;
    private GamepadEx driverGamepad;
    private Pose2d drivePowers;
    private ToggleButtonReader lockHeadingReader;
    private boolean lockHeading = false;
    private PIDFController headingController;
    private double currentHeading;
    private double turnAngle;
    private double targetHeading = 0;
    private double headingDeviation = 0;
    private Button clawButton;
    private double rightDistance = 0;
    private double wingAngle = -45;
    private double headingOutput = 0;
    private double headingDelayMS = 200;
    private double targetHeadingTime = 0;

    //This method runs once when the init button is pressed on the driver hub
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startI2CThread();

        driverGamepad = new GamepadEx(gamepad1);

        clawButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        clawButton.whenPressed(new ClawCommand(robot));

        drivePowers = new Pose2d();
        lockHeadingReader = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.X);
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        headingController.setTargetPosition(0);
        turnAngle = 0;
        robot.controls = Robot.ControlType.FIELDCENTRIC;

    }

    //After init is complete this method runs repeatably after init until the play button is pressed
    @Override
    public void init_loop() {
        telemetry.addData("right distance", robot.rightDistance);
    }

    //This method runs once after the start button is pressed
    @Override
    public void start() {
        robot.imu.resetYaw();
        robot.timer.reset();
        if (robot.alliance == Robot.Alliance.RED) {
            wingAngle = -135;
        }
    }

    //After start is complete this method runs repeatably after the start button is pressed
    @Override
    public void loop() {
        double startTime = robot.timer.milliseconds();
        driverGamepad.readButtons();
        lockHeadingReader.readValue();
        lockHeading = lockHeadingReader.getState();


        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        //Can get current heading from 3 different sensors
        //Odometry pods, built in imu, or external navx gyro
        //currentHeading = poseEstimate.getHeading();
        //currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentHeading = Math.toRadians(robot.navxHeading + DataStorage.finalAutoHeading);

        double gamely = driverGamepad.getLeftY();
        double gamelx = driverGamepad.getLeftX();
        double gamerx = driverGamepad.getRightX();


        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || driverGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            targetHeading = 90.0;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)|| driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            targetHeading = wingAngle;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y) && driverGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            robot.imu.resetYaw();
            robot.navxMicro.initialize();
            robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            targetHeading = 0;
            DataStorage.finalAutoHeading = 0;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            robot.clawSubsystem.toggle();
        }

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
            (gamely * Math.abs(gamely)),
            -(gamelx * Math.abs(gamelx))
        ).rotated(-currentHeading);

        if (gamerx > 0.05 || gamerx < -0.05) {
            targetHeading = Math.toDegrees(currentHeading);
            //Manually rotate output based on gamepad right stick
            headingOutput = -(gamerx * Math.abs(gamerx));
            targetHeadingTime = startTime;
        } else if ((startTime - targetHeadingTime) < headingDelayMS) {
            targetHeading = Math.toDegrees(currentHeading);
            headingOutput = -(gamerx * Math.abs(gamerx));
        } else {
            //Calculate the difference between our current heading and target
            //This is used with the PID so that we do not have to deal with angle wrap
            //The pid target is 0 degrees and we just give it how far off we are
            headingDeviation = Math.toDegrees(currentHeading) - targetHeading;
            //Still have to angle wrap our deviation because it can end up being greater
            //Than 180 degrees and try to go the wrong direction
            //headingDeviation = angleWrap(Math.toRadians(headingDeviation));
            headingDeviation = Angle.normDelta(Math.toRadians(headingDeviation));


            //headingController.setTargetPosition(0);
            headingOutput = (headingController.update(headingDeviation) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH;
        }
        drivePowers = new Pose2d(input, headingOutput);


        robot.drive.setWeightedDrivePower(drivePowers);
        robot.drive.update();

        //HuskyLens.Block[] blocks = robot.huskyLens.blocks();

        double elapsedtime = robot.timer.milliseconds() - startTime;
        // Print pose to telemetry
        telemetry.addData("loop ms", elapsedtime);
        telemetry.addData("mode", robot.controls);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("current heading", Math.toDegrees(currentHeading));
        telemetry.addData("locked heading", lockHeading);
        telemetry.addData("target heading", targetHeading);
        telemetry.addData("heading deviation", Math.toDegrees(headingDeviation));
        telemetry.addData("builtin IMU",robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) );
        telemetry.addData("claw state", robot.clawSubsystem.getState());
        telemetry.addData("alliance", robot.alliance);
        telemetry.addData("right distance", robot.rightDistance);
        telemetry.addData("heading offset", DataStorage.finalAutoHeading);
        //for (int i = 0; i < blocks.length; i++) {
        //    telemetry.addData("Block", blocks[i].toString());
        //}
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    //This runs when the stop button is pressed on the driver hub

    @Override
    public void stop() {
        robot.stopThread = true;
    }


}
