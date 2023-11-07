package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorage;

@Autonomous(name = "Auto Test Traj Build")

public class AutoTestOpMode extends OpMode {
    private MecanumVelocityConstraint fastModeVel;
    private TrajectorySequence hometoboard;
    private TrajectorySequence hometoboarda;
    private Trajectory hometoboard1;
    private Trajectory hometoboard2;
    private Trajectory boardtosomewhere;
    private Trajectory hometoboardT;
    public Robot robot;
    public Pose2d startPose;

    @Override
    public void init() {
        startPose = new Pose2d(-63, -40, Math.toRadians(0));
        robot = new Robot(hardwareMap);
        fastModeVel = new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH);

        hometoboarda = robot.drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();


/*
        Trajectory traj4 = robot.drive.trajectoryBuilder(startPose)
                .strafeRight(5)
                .strafeRight(5)
                .build();
        TrajectorySequence test3 = robot.drive.trajectorySequenceBuilder(startPose)
                .addTrajectory(traj4)
                .build();
        TrajectorySequence test4 robot.drive.trajectorySequenceBuilder(startPose)
                .addTrajectory(test3)
                .build();
                */
        /*
        hometoboard = robot.drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();
        */
        hometoboardT = robot.drive.trajectoryBuilder(startPose)

                .splineTo(new Vector2d(-32.00, -40.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(0, 12.00), Math.toRadians(50.00))
                .splineToSplineHeading(new Pose2d(37.00, 42, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();


        boardtosomewhere = robot.drive.trajectoryBuilder(hometoboardT.end())
                .addTemporalMarker(1, () ->{robot.clawSubsystem.toggle();})
                .lineToConstantHeading(new Vector2d(58, 20))
                .build();


        hometoboard1 = robot.drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();

        hometoboard2 = robot.drive.trajectoryBuilder(hometoboard1.end())
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .addTemporalMarker(2, () ->{robot.clawSubsystem.toggle();})
                .build();
/*
        hometoboard = robot.drive.trajectorySequenceBuilder(hometoboard1.start())
                .addTrajectory(hometoboard1)
                .waitSeconds(5)
                .addTrajectory(hometoboard2)
                .build();
*/
        hometoboard = robot.drive.trajectorySequenceBuilder(hometoboard1.start())
                .setVelConstraint(fastModeVel)
                .addTrajectory(hometoboardT)
                //.waitSeconds(5)
                .addTrajectory(boardtosomewhere)
                .build();

    }

    @Override
    public void start() {
        robot.drive.setPoseEstimate(startPose);
        robot.drive.followTrajectorySequenceAsync(hometoboard);
    }

    @Override
    public void loop() {
        robot.drive.update();
        telemetry.addData("x", robot.drive.getPoseEstimate().getX());
        telemetry.addData("y", robot.drive.getPoseEstimate().getY());
        telemetry.addData("clawPos", robot.clawServo.getPosition());
    }

    @Override
    public void stop() {
        DataStorage.finalAutoHeading = robot.navxHeading;
    }
}
