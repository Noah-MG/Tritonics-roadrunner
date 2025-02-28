package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "testing")
public final class testing extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-10, 65, Math.toRadians(90));

    public static double x_offset = -4;

    @Override
    public void runOpMode(){

        System.Arm arm = new System.Arm(this, true);
        System.Slides slides = new System.Slides(this, true);
        System.Intake intake = new System.Intake(this, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if(isStopRequested()){return;}

        Actions.runBlocking(intake.lowerPaddle());

        sleep(2000);
    }
}
