package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10, 65, Math.toRadians(90)))
                        .lineTo(new Vector2d(-5, 32))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-22, 49, Math.toRadians(230)), Math.toRadians(-90))
                        .turn(Math.toRadians(-80))
                        .lineToLinearHeading( new Pose2d(-30, 49, Math.toRadians(230)))
                        .turn(Math.toRadians(-80))
                        .splineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-3, 32, Math.toRadians(91)))
                        .lineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-1, 32, Math.toRadians(91)))
                        .lineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(1, 32, Math.toRadians(91)))
                        .lineTo(new Vector2d(-40, 60))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}