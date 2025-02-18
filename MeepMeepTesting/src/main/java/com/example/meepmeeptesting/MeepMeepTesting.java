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
                        .splineToLinearHeading(new Pose2d(-35, 27, Math.toRadians(180)), Math.toRadians(-90))
                        .lineTo(new Vector2d(-45, 60))
                        .splineToConstantHeading(new Vector2d(-45, 27), Math.toRadians(90))
                        .lineTo(new Vector2d(-48, 60))
                        .splineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-3, 32, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-1, 32, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-40, 65, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(1, 32, Math.toRadians(90)))
                        .lineTo(new Vector2d(-40, 60))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}