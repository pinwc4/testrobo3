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
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;

@Autonomous(name = "Test 4 Red")
public class autotest4red extends OpMode {
    public Robot robot;
    public Pose2d startPose;
    public MecanumVelocityConstraint fastModeVel;
    public TrajectorySequence hometoboard;
    public TrajectorySequence boardtosomewhere;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        startPose = new Pose2d(-63, -40, Math.toRadians(0));

        fastModeVel = new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH);


        hometoboard = robot.drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(fastModeVel)
                .splineTo(new Vector2d(-24.00, -40.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(6, 12.00), Math.toRadians(50.00))
                .splineToSplineHeading(new Pose2d(37.00, 45, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();

        boardtosomewhere = robot.drive.trajectorySequenceBuilder(hometoboard.end())
                .lineToConstantHeading(new Vector2d(60, 20))
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
        robot.drive.followTrajectorySequenceAsync(boardtosomewhere);
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
