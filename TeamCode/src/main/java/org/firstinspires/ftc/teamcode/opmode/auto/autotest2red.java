package org.firstinspires.ftc.teamcode.opmode.auto;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Disabled
@Autonomous(name = "Test 2 Red")

public class autotest2red extends LinearOpMode {

    public void runOpMode() throws InterruptedException{

        /*
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-63, -40, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence hometoboard = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-24.00, -40.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(6, 12.00), Math.toRadians(60.00))
                .splineToSplineHeading(new Pose2d(37.00, 46.60, Math.toRadians(90.00)), Math.toRadians(50.00))
                .build();

        waitForStart();

        drive.followTrajectorySequence(hometoboard);
        drive.update();

         */
    }

}
