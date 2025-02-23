package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "1+3 Specimen")
public final class OnePThreeSpeci extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-10, 65, Math.toRadians(90));

    @Override
    public void runOpMode(){

        System.Arm arm = new System.Arm(this);
        System.Slides slides = new System.Slides(this);
        System.Intake intake = new System.Intake(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if(isStopRequested()){return;}

        Actions.runBlocking( new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(initialPose)
                                .strafeTo(new Vector2d(-3, 33))
                                .build(),
                        arm.closeSpecimen(),
                        slides.raiseSlides()
                ),
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .strafeToLinearHeading(new Vector2d(-22, 52), Math.toRadians(230))
                                .build(),
                        slides.lowerSlides(),
                        new SequentialAction(
                                slides.extendSlides(),
                                intake.lowerPaddle()
                        )
                ),
                arm.openSpecimen(),
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-80))
                        .strafeToLinearHeading(new Vector2d(-30, 52), Math.toRadians(230))
                        .turn(Math.toRadians(-80))
                        .build(),
                new RaceAction(
                        drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-90)), Math.toRadians(90))
                            .build(),
                        slides.retractSlides(),
                        intake.raiseClaw()
                ),
                new ParallelAction(
                        arm.closeSpecimen(),
                        slides.raiseSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-3, 33, Math.toRadians(90)), Math.toRadians(-90))
                                .build()
                ),
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
                ),
                new ParallelAction(
                        arm.closeSpecimen(),
                        slides.raiseSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-1, 33, Math.toRadians(90)), Math.toRadians(-90))
                                .build()
                ),
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
                ),
                new ParallelAction(
                        arm.closeSpecimen(),
                        slides.raiseSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(1, 33, Math.toRadians(90)), Math.toRadians(-90))
                                .build()
                ),
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(-40, 60))
                                .build(),
                        slides.lowerSlides()
                )
        ));
    }
}
