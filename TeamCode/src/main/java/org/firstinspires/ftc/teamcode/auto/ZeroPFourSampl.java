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
@Autonomous(name = "0+4 Sample")
public class ZeroPFourSampl extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-10, 65, Math.toRadians(90));

    @Override
    public void runOpMode() {

        System.Arm arm = new System.Arm(this, false);
        System.Slides slides = new System.Slides(this, false);
        System.Intake intake = new System.Intake(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action handoff = new ParallelAction();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(new SequentialAction(
            new ParallelAction(
                drive.actionBuilder(initialPose)
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(50, 50, Math.toRadians(215)), Math.toRadians(45))
                    .build(),
                    slides.raiseSlides(),
                    new SequentialAction(
                            new SleepAction(2),
                            arm.raiseArm()
                    )
            ),
            arm.openOuttake(),
            new SleepAction(0.5),
            new ParallelAction(
                arm.closeOuttake(),
                arm.lowerArm(),
                slides.lowerSlides(),
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(48, 41), Math.toRadians(-90))
                        .build(),
                slides.extendSlides(),
                new SequentialAction(
                    new SleepAction(0.7),
                    intake.lowerClaw(),
                    intake.openIntake()
                )
            )
        ));
    }
}
