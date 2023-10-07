package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-63.00, -40.00, Math.toRadians(0.00)))
                                .splineTo(new Vector2d(-20.00, -40.00), Math.toRadians(-0.89))
                                .splineTo(new Vector2d(9.95, 12.00), Math.toRadians(60.00))
                                .splineToSplineHeading(new Pose2d(37.00, 46.60, Math.toRadians(90.00)), Math.toRadians(50.00))
                                .build()
                );


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