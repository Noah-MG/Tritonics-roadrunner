package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action path1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-3, 33))
//                .strafeToLinearHeading(new Vector2d(-22, 52), Math.toRadians(230))
//                .turn(Math.toRadians(-80))
//                .strafeToLinearHeading(new Vector2d(-30, 52), Math.toRadians(230))
//                .turn(Math.toRadians(-80))
//                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-90)), Math.toRadians(90))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-3, 33, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-1, 33, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(1, 33, Math.toRadians(90)), Math.toRadians(-90))
//                .strafeTo(new Vector2d(-40, 60))
//                .build();

        waitForStart();

        if(isStopRequested()){return;}

        Actions.runBlocking( new SequentialAction(
                new ParallelAction(
                        path1
                )
        ));
    }
}
