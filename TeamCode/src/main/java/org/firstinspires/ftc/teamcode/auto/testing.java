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

    public static double intakePos = 0.5;
    public static double outtakePos = 0.5;
    public static double speciPos = 0.5;

    @Override
    public void runOpMode(){

        Ports.Builder builder = new Ports.Builder();
        builder.allActive = true;
        Ports ports = new Ports(this, builder);

        waitForStart();

        if(isStopRequested()){return;}

        while(opModeIsActive()){
            ports.intakeClaw.setPosition(intakePos);
            ports.outtakeClaw.setPosition(outtakePos);
            ports.specimenClaw.setPosition(speciPos);
        }

    }
}
