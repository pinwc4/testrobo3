package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "Test 5 Red")
public class AutoTest5Red  extends OpMode {
    public Robot robot;
    public Pose2d startPose;

    public TrajectorySequence hometoboard;
    public TrajectorySequence boardtosomewhere;
    public TrajectorySequence traj1;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        hometoboard = robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-32.00, -40.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(0, 12.00), Math.toRadians(50.00))
                .splineToSplineHeading(new Pose2d(37.00, 42, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();

        traj1 = hometoboard;

        boardtosomewhere = robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> robot.drive.followTrajectorySequence(hometoboard))
                .build();


    }

    @Override
    public void loop() {

    }

}
