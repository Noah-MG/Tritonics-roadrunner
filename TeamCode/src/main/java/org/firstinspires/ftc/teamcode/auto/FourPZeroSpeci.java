package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "4+0 Specimen")
public final class FourPZeroSpeci extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-10, 65, Math.toRadians(90));

    public static double x_offset = -6;

    public static double tvc = 50.0;
    public static double avc = 3.14;
    public static double minac = -110;
    public static double maxac = 110;

//    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
//            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
//    public final VelConstraint defaultVelConstraint =
//            new MinVelConstraint(Arrays.asList(
//                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
//                    new AngularVelConstraint(PARAMS.maxAngVel)
//            ));
//    public final AccelConstraint defaultAccelConstraint =
//            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);


    VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
            MecanumDrive.kinematics.new WheelVelConstraint(tvc),
            new AngularVelConstraint(avc)
    ));
    AccelConstraint accelConstraint = new ProfileAccelConstraint(minac, maxac);

    @Override
    public void runOpMode(){

        System.Arm arm = new System.Arm(this, true);
        System.Slides slides = new System.Slides(this, true);
        System.Intake intake = new System.Intake(this, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Ports.Builder builder = new Ports.Builder();
        builder.allActive = true;
        Ports ports = new Ports(this, builder);

        waitForStart();

        if(isStopRequested()){return;}

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(initialPose)
                                .strafeTo(new Vector2d(-3, 34))
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
                                .strafeToLinearHeading(new Vector2d(-30+x_offset, 47), Math.toRadians(250))
                                .build(),
                        slides.lowerSlides(),
                        new SequentialAction(
                                new SleepAction(1.5),
                                slides.extendSlides(),
                                intake.lowerPaddle()
                        )
                ));

        ports.lsh_l.setPower(0);
        ports.lsh_r.setPower(0);

        Actions.runBlocking( new SequentialAction(
                arm.openSpecimen(),
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-120))
                        .build()
        ));

        Actions.runBlocking(new ParallelAction(
                intake.raiseClaw(),
                drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(Math.toRadians(14))
                        .strafeToLinearHeading(new Vector2d(-39+x_offset, 47), Math.toRadians(250))
                        .build()
        ));

        Actions.runBlocking( new SequentialAction(
                intake.lowerPaddle(),
                drive.actionBuilder(drive.localizer.getPose())
                        .waitSeconds(0.4)
                        .turn(Math.toRadians(-120))
                        .build()
        ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-90)), Math.toRadians(90), velConstraint, accelConstraint)
                            .build(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                slides.retractSlides()
                        ),
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
                                    .splineToLinearHeading(new Pose2d(-2, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                    .build()))
                ));

//        FASTER MOTION STARTS HERE

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90), velConstraint, accelConstraint)
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
                                                .splineToLinearHeading(new Pose2d(1, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                                .build()))
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90), velConstraint, accelConstraint)
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
                                                .splineToLinearHeading(new Pose2d(3, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                                .build()))
                ));

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(-40, 60), velConstraint, accelConstraint)
                                .build(),
                        slides.lowerSlides()
                )
        );
    }
}
