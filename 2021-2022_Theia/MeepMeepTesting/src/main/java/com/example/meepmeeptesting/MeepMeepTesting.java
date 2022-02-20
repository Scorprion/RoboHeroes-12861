package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(183.6), Math.toRadians(162.272), 10.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -64, Math.toRadians(180)))
                                .strafeRight(10)
                                .lineToLinearHeading(new Pose2d(-14, -38, Math.toRadians(270)))
                                .splineTo(new Vector2d(5, -60), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(30 + 15, -64), Math.toRadians(0))
                                .back(30 - 10)
                                .splineTo(new Vector2d(-14 , -38), Math.toRadians(90))
                                .splineTo(new Vector2d(15, -66), Math.toRadians(0))
                                .splineTo(new Vector2d(45, -64), Math.toRadians(0))



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}