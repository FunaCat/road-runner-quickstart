package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-43.31, 59.92, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(-45.01, 26.71), Math.toRadians(-88.06))
                        .splineTo(new Vector2d(-43.88, -7.45), Math.toRadians(-88.10))
                        .splineTo(new Vector2d(-39.92, -45.96), Math.toRadians(-84.12))
                        .splineTo(new Vector2d(-3.87, -56.71), Math.toRadians(-16.62))
                        .splineTo(new Vector2d(39.35, -55.58), Math.toRadians(1.50))
                        .splineTo(new Vector2d(39.92, -3.68), Math.toRadians(89.37))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}