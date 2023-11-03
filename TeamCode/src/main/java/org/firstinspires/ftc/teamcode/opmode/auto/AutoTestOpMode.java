package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Test 1 Red")

public class AutoTestOpMode extends OpMode {
    TrajectorySequence hometoboard;
    public Robot robot;
    Pose2d startPose = new Pose2d(-67, -40, Math.toRadians(0));

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        /*
        TrajectorySequence hometoboard = robot.drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();

         */
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
        Trajectory hometoboard = robot.drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();
        */

        Trajectory hometoboard1 = robot.drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();

        Trajectory hometoboard2 = robot.drive.trajectoryBuilder(hometoboard1.end())
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .addTemporalMarker(2, () ->{robot.clawSubsystem.toggle();})
                .build();

        TrajectorySequence hometoboard = robot.drive.trajectorySequenceBuilder(hometoboard1.start())
                .addTrajectory(hometoboard1)
                .waitSeconds(5)
                .addTrajectory(hometoboard2)
                .build();

    }

    @Override
    public void start() {
        robot.drive.followTrajectorySequence(hometoboard);
    }

    @Override
    public void loop() {
        telemetry.addData("x", robot.drive.getPoseEstimate().getX());
        telemetry.addData("y", robot.drive.getPoseEstimate().getY());
        telemetry.addData("clawPos", robot.clawServo.getPosition());
    }

}
