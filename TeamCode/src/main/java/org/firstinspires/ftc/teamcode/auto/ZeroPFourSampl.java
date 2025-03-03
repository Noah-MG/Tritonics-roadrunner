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
    Pose2d initialPose = new Pose2d(40, 65, Math.toRadians(90));

    @Override
    public void runOpMode() {
        System.Arm arm = new System.Arm(this, false);
        System.Slides slides = new System.Slides(this, false);
        System.Intake intake = new System.Intake(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Ports.Builder builder = new Ports.Builder();
        builder.allActive = true;
        Ports ports = new Ports(this, builder);

        Action handoff = new SequentialAction(
                intake.raiseClaw(),
                intake.squareIntake(),
                intake.loosenIntake(),
                arm.openOuttake(),
                arm.partial(),
                new SleepAction(0.3),
                new RaceAction(
                        new ParallelAction(
                                slides.lowerSlides(),
                                slides.retractSlides()
                        ),
                        new SleepAction(1.3)
                ),
                arm.closeOuttake(),
                new SleepAction(0.6),
                intake.openIntake());

        Action handoff2 = new SequentialAction(
                intake.raiseClaw(),
                intake.squareIntake(),
                intake.loosenIntake(),
                arm.openOuttake(),
                arm.partial(),
                new SleepAction(0.3),
                new RaceAction(
                        new ParallelAction(
                                slides.lowerSlides(),
                                slides.retractSlides()
                        ),
                        new SleepAction(1.3)
                ),
                arm.closeOuttake(),
                new SleepAction(0.6),
                intake.openIntake());

        Action handoff3 = new SequentialAction(
                intake.raiseClaw(),
                intake.squareIntake(),
                intake.loosenIntake(),
                arm.openOuttake(),
                arm.partial(),
                new SleepAction(0.3),
                new RaceAction(
                    new ParallelAction(
                            slides.lowerSlides(),
                            slides.retractSlides()
                    ),
                    new SleepAction(1.3)
                ),
                arm.closeOuttake(),
                new SleepAction(0.6),
                intake.openIntake());

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        ports.intakePitch.setPosition(0.55);

        Actions.runBlocking(
            new ParallelAction(
                intake.squareIntake(),
                intake.closeIntake(),
                drive.actionBuilder(initialPose)
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(57.5, 59, Math.toRadians(215)), Math.toRadians(45))
                    .build(),
                slides.raiseSlides(),
                new SequentialAction(
                        new SleepAction(1),
                        arm.raiseArm()
                ),
                arm.closeOuttake()
            ));

        sleep(1000);

        Actions.runBlocking(new SequentialAction(
                arm.openOuttake(),
                new SleepAction(0.3),
                new ParallelAction(
                        arm.openOuttake(),
                        arm.lowerArm(),
                        slides.lowerSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(55, 49), Math.toRadians(-90))
                                .build(),
                        slides.extendSlides(),
                        new SequentialAction(
                                new SleepAction(0.7),
                                intake.lowerClaw(),
                                intake.openIntake()
                        )),
                new SleepAction(0.2),
                intake.closeIntake(),
                new SleepAction(0.2),
                handoff
        ));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(56.5, 58, Math.toRadians(215)), Math.toRadians(45))
                                .build(),
                        slides.raiseSlides(),
                        new SequentialAction(
                                new SleepAction(2),
                                arm.raiseArm()
                        )
                ),
                new SleepAction(1),
                arm.openOuttake(),
                intake.closeIntake()
        ));

        ports.intakePitch.setPosition(0.55);
        sleep(1000);

        Actions.runBlocking(new SequentialAction(
                arm.openOuttake(),
                new SleepAction(0.3),
                new ParallelAction(
                        arm.openOuttake(),
                        arm.lowerArm(),
                        slides.lowerSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(67, 49), Math.toRadians(-80))
                                .build(),
                        slides.extendSlides(),
                        new SequentialAction(
                                new SleepAction(0.7),
                                intake.lowerClaw(),
                                intake.openIntake()
                        )),
                intake.squareIntake(),
                new SleepAction(0.2),
                intake.closeIntake(),
                new SleepAction(0.2),
                handoff2
        ));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(56.5, 57, Math.toRadians(215)), Math.toRadians(45))
                                .build(),
                        slides.raiseSlides(),
                        new SequentialAction(
                                new SleepAction(2),
                                arm.raiseArm()
                        )
                ),
                new SleepAction(0.7)
        ));

        sleep(1200);

        Actions.runBlocking(new SequentialAction(
                arm.openOuttake(),
                new SleepAction(0.3),
                new ParallelAction(
                        arm.openOuttake(),
                        arm.lowerArm(),
                        slides.lowerSlides(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(55, 32), Math.toRadians(0))
                                .build(),
                        slides.extendSlides(),
                        intake.rotateIntake(),
                        new SequentialAction(
                                new SleepAction(0.7),
                                intake.lowerClaw(),
                                intake.openIntake()
                        )),
                new SleepAction(0.2),
                intake.closeIntake(),
                new SleepAction(0.2),
                handoff3
        ));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(56.5, 58, Math.toRadians(215)), Math.toRadians(45))
                                .build(),
                        slides.raiseSlides(),
                        new SequentialAction(
                                new SleepAction(2),
                                arm.raiseArm()
                        )
                ),
                new SleepAction(1.2),
                arm.openOuttake(),
                new SleepAction(1),
                arm.lowerArm(),
                slides.lowerSlides()
        ));

        sleep(1000);

    }
}
