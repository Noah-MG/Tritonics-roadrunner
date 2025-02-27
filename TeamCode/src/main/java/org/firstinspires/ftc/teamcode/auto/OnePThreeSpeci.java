package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "1+3 Specimen")
public final class OnePThreeSpeci extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-10, 65, Math.toRadians(90));

    public static double x_offset = -2;

    @Override
    public void runOpMode(){

        System.Arm arm = new System.Arm(this, true);
        System.Slides slides = new System.Slides(this, true);
        System.Intake intake = new System.Intake(this, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if(isStopRequested()){return;}

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(initialPose)
                                .strafeTo(new Vector2d(-3, 33))
                                .build(),
                        arm.closeSpecimen(),
                        slides.raiseSlides(),
                        intake.raiseClaw(),
                        intake.closeIntake()
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .strafeToLinearHeading(new Vector2d(-30+x_offset, 47), Math.toRadians(240))
                                .build(),
                        slides.lowerSlides(),
                        new SequentialAction(
                                new SleepAction(1.5),
                                slides.extendSlides(),
                                intake.lowerPaddle()
                        )
                ));

        Actions.runBlocking( new SequentialAction(
                arm.openSpecimen(),
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-110))
                        .strafeToLinearHeading(new Vector2d(-41+x_offset, 47), Math.toRadians(240))
                        .turn(Math.toRadians(-110))
                        .build()
        ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-90)), Math.toRadians(90))
                            .build(),
                        slides.retractSlides(),
                        intake.raiseClaw()
                ));

        Actions.runBlocking(
                new ParallelAction(
                        arm.closeSpecimen(),
                        new SequentialAction(
                            new SleepAction(0.5),
                            new ParallelAction(
                            slides.raiseSlides(),
                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(-1, 33, Math.toRadians(90)), Math.toRadians(-90))
                                    .build()))
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90))
                                .build(),
                        new SequentialAction(
                                slides.lowerSlides(),
                                arm.openSpecimen()
                        )
                ));

        Actions.runBlocking(
                new ParallelAction(
                        arm.closeSpecimen(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                new ParallelAction(
                                        slides.raiseSlides(),
                                        drive.actionBuilder(drive.localizer.getPose())
                                                .setTangent(Math.toRadians(-90))
                                                .splineToLinearHeading(new Pose2d(1, 33, Math.toRadians(90)), Math.toRadians(-90))
                                                .build()))
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90))
                                .build(),
                        new SequentialAction(
                                slides.lowerSlides(),
                                arm.openSpecimen()
                        )
                ));

        Actions.runBlocking(
                new ParallelAction(
                        arm.closeSpecimen(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                new ParallelAction(
                                        slides.raiseSlides(),
                                        drive.actionBuilder(drive.localizer.getPose())
                                                .setTangent(Math.toRadians(-90))
                                                .splineToLinearHeading(new Pose2d(2, 33, Math.toRadians(90)), Math.toRadians(-90))
                                                .build()))
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(-40, 60))
                                .build(),
                        slides.lowerSlides()
                )
        );
    }
}
