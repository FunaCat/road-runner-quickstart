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
                        drive.trajectorySequenceBuilder(new Pose2d(-38.78, 58.41, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-38.97, 19.53), Math.toRadians(269.72))
                                .splineTo(new Vector2d(-39.54, 1.04), Math.toRadians(-85.06))
                                .splineTo(new Vector2d(-41.24, -15.19), Math.toRadians(-65.34))
                                .splineTo(new Vector2d(-35.20, -32.18), Math.toRadians(-70.43))
                                .splineTo(new Vector2d(-33.50, -52.75), Math.toRadians(-85.28))
                                .splineTo(new Vector2d(-52.94, -47.09), Math.toRadians(163.76))
                                .splineTo(new Vector2d(-58.60, -28.40), Math.toRadians(106.86))
                                .splineTo(new Vector2d(-59.36, 0.28), Math.toRadians(91.51))
                                .splineTo(new Vector2d(-43.31, -0.28), Math.toRadians(-2.02))
                                .splineTo(new Vector2d(-20.85, 0.09), Math.toRadians(0.96))
                                .splineTo(new Vector2d(9.72, 2.74), Math.toRadians(4.94))
                                .splineTo(new Vector2d(36.71, 3.11), Math.toRadians(0.80))
                                .splineTo(new Vector2d(37.65, -9.15), Math.toRadians(-85.60))
                                .splineTo(new Vector2d(40.29, -40.29), Math.toRadians(-85.15))
                                .splineTo(new Vector2d(37.27, -63.13), Math.toRadians(262.47))
                                .splineTo(new Vector2d(53.88, -62.19), Math.toRadians(3.25))
                                .splineTo(new Vector2d(51.24, -45.20), Math.toRadians(98.84))
                                .splineTo(new Vector2d(56.71, -41.24), Math.toRadians(35.91))
                                .splineTo(new Vector2d(59.36, -15.57), Math.toRadians(60.64))
                                .splineTo(new Vector2d(59.54, -5.94), Math.toRadians(-64.33))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}