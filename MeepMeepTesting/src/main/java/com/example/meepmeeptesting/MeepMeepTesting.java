package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Objects;

import javax.imageio.ImageIO;

public class MeepMeepTesting {



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        AddTrajectorySequenceCallback blue2red = drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-63.00, -40.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(-20.00, -40.00), Math.toRadians(0.00))
                        .splineTo(new Vector2d(9.95, 12.00), Math.toRadians(60.00))
                        .splineToSplineHeading(new Pose2d(37.00, 46.60, Math.toRadians(90.00)), Math.toRadians(50.00))
                        .build();

        AddTrajectorySequenceCallback junk = drive ->
                drive.trajectorySequenceBuilder(new Pose2d(36.45, 51.93, Math.toRadians(270.00)))
                        .UNSTABLE_addTemporalMarkerOffset(7.94,() -> {})
                        .lineToSplineHeading(new Pose2d(0.24, 22.99, Math.toRadians(221.16)))
                        .splineTo(new Vector2d(-0.36, -22.86), Math.toRadians(269.24))
                        .build();

        AddTrajectorySequenceCallback blue2red2 = drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-63.00, -40.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(-24.00, -40.00), Math.toRadians(0.00))
                        .splineToConstantHeading(new Vector2d(6, 12.00), Math.toRadians(50.00))
                        .splineToSplineHeading(new Pose2d(37.00, 45, Math.toRadians(90.00)), Math.toRadians(50.00))
                        .turn(Math.toRadians(-90))
                        .build();

        AddTrajectorySequenceCallback test2 = drive -> drive.trajectorySequenceBuilder(new Pose2d(-35.92, -71.83, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-35.92, -26.62), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-40.44, -37.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .splineToSplineHeading(new Pose2d(18.17, -37.00, Math.toRadians(1.21)), Math.toRadians(1.21))
                .build();

        AddTrajectorySequenceCallback test3 = drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-38.35, 63.30, Math.toRadians(270.00)))
                        .lineToLinearHeading(new Pose2d(-47.00, 30.00, Math.toRadians(250.00)))
                        .setReversed(true)
                        .waitSeconds(2)
                        .splineToConstantHeading(new Vector2d(-43.00, 41.00), Math.toRadians(70.00))
                        .splineToLinearHeading(new Pose2d(-39.00, 59.00, Math.toRadians(0.00)), Math.toRadians(45.00))
                        .setReversed(false)
                        .splineTo(new Vector2d(-27.00, 59.00), Math.toRadians(0.00))
                        .splineTo(new Vector2d(13.00, 59.00), Math.toRadians(0.00))
                        .splineTo(new Vector2d(27.83, 36.89), Math.toRadians(-56.15))
                        .splineTo(new Vector2d(30.16, 7.28), Math.toRadians(-85.51))
                        .build();


        AddTrajectorySequenceCallback test4 = drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-39.00, 63.00, Math.toRadians(270.00)))
                        .splineToLinearHeading(new Pose2d(-47.00, 30.00, Math.toRadians(250.00)), Math.toRadians(250.00))
                        .setReversed(true)
                        .splineTo(new Vector2d(-43.00, 41.00), Math.toRadians(70.00))
                        .splineTo(new Vector2d(-38.23, 55.61), Math.toRadians(45.00))
                        .splineTo(new Vector2d(-28.20, 58.91), Math.toRadians(0.00))
                        .splineTo(new Vector2d(11.32, 58.91), Math.toRadians(0.00))
                        .splineToLinearHeading(new Pose2d(34.07, 37.74, Math.toRadians(180.00)), Math.toRadians(-41.83))
                        .splineToLinearHeading(new Pose2d(48.39, 32.48, Math.toRadians(180.00)), Math.toRadians(0.00))
                        .build();


        AddTrajectorySequenceCallback[] trajArray = {blue2red, junk};

/*
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(blue2red2);
*/
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(test4);

        /*
        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/centerstagerotated.png"));
        } catch (IOException e) {
        }
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
        //meepMeep.setBackground(img)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot2)
                .start();
    }
}