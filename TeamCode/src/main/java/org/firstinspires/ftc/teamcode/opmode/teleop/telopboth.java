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
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;


@TeleOp(name="Claw Opmode")
public class telopboth  extends OpMode {
    private Robot robot;
    private GamepadEx driverGamepad;
    private Pose2d drivePowers;
    private ToggleButtonReader lockHeadingReader;
    private boolean lockHeading = false;
    private PIDFController headingController;
    private double currentHeading;
    private double turnAngle;
    private double targetHeading = 90;
    private double headingDeviation = 0;
    private Button clawButton;
    private double rightDistance = 0;


    //This method runs once when the init button is pressed on the driver hub
    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);

        clawButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        clawButton.whenPressed(new ClawCommand(robot));

        drivePowers = new Pose2d();
        lockHeadingReader = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.X);
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
        turnAngle = 0;

    }

    //After init is complete this method runs repeatably after init until the play button is pressed
    @Override
    public void init_loop() {
        rightDistance = robot.rightDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("right distance", rightDistance);
    }

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
        lockHeadingReader.readValue();
        lockHeading = lockHeadingReader.getState();


        Orientation angles = robot.navxgyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        //Can get current heading from 3 different sensors
        //Odometry pods, built in imu, or external navx gyro
        //currentHeading = poseEstimate.getHeading();
        //currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentHeading = Math.toRadians(angles.firstAngle);

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
            if (lockHeading) {
                //Ignore gamepad input if it is tiny to avoid noise accumulating
                if (gamerx > 0.05 || gamerx < -0.05) {
                    targetHeading = targetHeading + (-(gamerx * Math.abs(gamerx)) * 6);

                    if (targetHeading > 180) {
                        targetHeading = targetHeading - 360;
                    } else if (targetHeading < -180) {
                        targetHeading = targetHeading + 360;
                    }


                    //targetHeading = Math.toDegrees(angleWrap(Math.toRadians(targetHeading)));
                }

                //Calculate the differenc between our current heading and target
                //This is used with the PID so that we do not have to deal with angle wrap
                //The pid target is 0 degrees and we just give it how far off we are
                headingDeviation = Math.toDegrees(currentHeading) - targetHeading;

                //Still have to angle wrap our deviation because it can end up being greater
                //Than 180 degrees and try to go the wrong direction
                //headingDeviation = angleWrap(Math.toRadians(headingDeviation));
                headingDeviation = Angle.normDelta(Math.toRadians(headingDeviation));

                headingController.setTargetPosition(0);
                double headingInput = (headingController.update(headingDeviation) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH;

                //Do not do this, it does not handle angle wrap properly
                //headingController.setTargetPosition(Math.toRadians(targetHeading));
                //double headingInput = (headingController.update(currentHeading) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH;

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
            targetHeading = 90.0;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)|| driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            targetHeading = -135.0;
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y) && driverGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            robot.imu.resetYaw();
            robot.navxMicro.initialize();
            robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        }

        if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            robot.clawSubsystem.toggle();
        }

        robot.drive.setWeightedDrivePower(drivePowers);
        robot.drive.update();



        double elapsedtime = robot.timer.milliseconds() - starttime;
        // Print pose to telemetry
        telemetry.addData("loop ms", elapsedtime);
        telemetry.addData("mode", robot.controls);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("current heading", Math.toDegrees(currentHeading));
        telemetry.addData("pose heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("navx heading", angles.firstAngle);
        telemetry.addData("imu heading", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("locked heading", lockHeading);
        telemetry.addData("target heading", targetHeading);
        telemetry.addData("heading deviation", Math.toDegrees(headingDeviation));
        telemetry.addData("claw state", robot.clawSubsystem.getState());
        telemetry.addData("claw period", robot.clawSubsystem.period);
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    //This runs when the stop button is pressed on the driver hub
    /*
    @Override
    public void stop() {

    }
    */

}
