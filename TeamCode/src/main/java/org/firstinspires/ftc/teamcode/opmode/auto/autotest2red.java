package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Test 2 Red")

public class autotest2red extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-63, -40, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence hometoboard = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-20.00, -40.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(9.95, 12.00), Math.toRadians(60.00))
                .splineToSplineHeading(new Pose2d(37.00, 46.60, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();

        waitForStart();

        drive.followTrajectorySequence(hometoboard);
        drive.update();
    }

}
