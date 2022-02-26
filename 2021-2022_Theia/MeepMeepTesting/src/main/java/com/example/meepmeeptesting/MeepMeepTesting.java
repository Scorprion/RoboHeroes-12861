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
                        drive.trajectorySequenceBuilder(new Pose2d(-12, -40, Math.toRadians(270)))
                                .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(-30))
                                .splineToLinearHeading(new Pose2d(20, -64, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(55, -64), Math.toRadians(0))

                                /*
                                .splineToSplineHeading(new Pose2d(-5, -58, Math.toRadians(0)), Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(12, -64, Math.toRadians(0)), Math.toRadians(0),
                                        JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                                        JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .lineToLinearHeading(new Pose2d(24, -64, Math.toRadians(0)))
                                .splineTo(new Vector2d(-12, -40), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-10, -55, Math.toRadians(0)), Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(5, -64, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(45, -64), Math.toRadians(0))
                                */


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}