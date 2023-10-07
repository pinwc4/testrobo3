package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-67.00, -40.00, Math.toRadians(0)))

                                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(55.00))
                                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(90.00))
                                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(60)), Math.toRadians(55))
                                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(90))
                                .build()
                );
/*
        TrajectorySequence hometoboard = drive.trajectorySequenceBuilder(new Pose2d(-67.00, -40.00, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-14.75, -42.64, Math.toRadians(60.00)), Math.toRadians(235.00))
                .splineToSplineHeading(new Pose2d(-4.77, -1.69, Math.toRadians(60.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(14.46, 18.72, Math.toRadians(45.43)), Math.toRadians(235))
                .splineToSplineHeading(new Pose2d(35.00, 40.00, Math.toRadians(90.00)), Math.toRadians(240))
                .build();
*/
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                //.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot2)
                .start();
    }
}