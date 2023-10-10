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
                        .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                        .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                        .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(55))
                        .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                        .build();



        AddTrajectorySequenceCallback[] trajArray = {blue2red, junk};


        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(blue2red);


        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/centerstagerotated.png"));
        } catch (IOException e) {
        }

        //meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                //.setDarkMode(true)
        meepMeep.setBackground(img)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot2)
                .start();
    }
}