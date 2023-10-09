package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;


@Autonomous(name = "Test 1 Red")

public class autotest1red extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence hometoboard = drive.trajectorySequenceBuilder(new Pose2d(64.37, 38.97, Math.toRadians(0)))
                .splineTo(new Vector2d(23.56, 40.00), Math.toRadians(198.43))
                .splineTo(new Vector2d(6.68, 26.06), Math.toRadians(234.96))
                .splineTo(new Vector2d(-3.89, -17.39), Math.toRadians(-88.90))
                .splineTo(new Vector2d(-35.45, -49.10), Math.toRadians(270.00))
                .build();


        waitForStart();

        drive.followTrajectorySequence(hometoboard);
        drive.update();
    }

}
