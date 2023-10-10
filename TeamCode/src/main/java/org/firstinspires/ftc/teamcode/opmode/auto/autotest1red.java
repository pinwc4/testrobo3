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

        Pose2d startPose = new Pose2d(-67, -40, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence hometoboard = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                .build();

        waitForStart();

        drive.followTrajectorySequence(hometoboard);
        drive.update();
    }

}
