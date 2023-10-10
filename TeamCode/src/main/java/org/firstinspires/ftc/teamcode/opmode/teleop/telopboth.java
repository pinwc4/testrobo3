package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Switchable Opmode")
public class telopboth  extends OpMode {
    private Robot robot;
    private GamepadEx driverGamepad;
    private Pose2d drivePowers;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);

        drivePowers = new Pose2d();
    }

    /*
    @Override
    public void init_loop() {

    }
    */

    @Override
    public void start() {
        robot.timer.reset();
    }

    @Override
    public void loop() {
        double starttime = robot.timer.milliseconds();
        driverGamepad.readButtons();
        Pose2d poseEstimate = robot.drive.getPoseEstimate();

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
                    -(gamely * Math.abs(gamely)),
                    -(gamelx * Math.abs(gamelx))
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drivePowers = new Pose2d(input.getX(), input.getY(), -(gamerx * Math.abs(gamerx)));

        } else {
            //Robot Centric driving
            drivePowers = new Pose2d(-(gamely * Math.abs(gamely)), -(gamelx * Math.abs(gamelx)), -(gamerx * Math.abs(gamerx)));

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
        telemetry.update();

    }

    /*
    @Override
    public void stop() {

    }
    */

}
