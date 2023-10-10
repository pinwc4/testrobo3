package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Test 3 Red")
public class autotest3red extends OpMode {
    public Robot robot;
    public Pose2d startPose;
    public TrajectorySequence hometoboard;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        startPose = new Pose2d(-63, -40, Math.toRadians(0));

        hometoboard = robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-24.00, -40.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(6, 12.00), Math.toRadians(60.00))
                .splineToSplineHeading(new Pose2d(37.00, 45, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();
    }

    /*
    @Override
    public void init_loop() {

    }
    */

    @Override
    public void start() {
        robot.drive.setPoseEstimate(startPose);
        robot.drive.followTrajectorySequence(hometoboard);
        robot.drive.update();
    }

    @Override
    public void loop() {
        robot.drive.update();
        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        Orientation angles = robot.navxgyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        
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
