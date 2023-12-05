package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class RedLeft {
    public TrajectorySequence hometoboard;

    public RedLeft(Robot robot, Pose2d startPose) {
        hometoboard = robot.drive.trajectorySequenceBuilder(startPose)
                .build();
    }
}
