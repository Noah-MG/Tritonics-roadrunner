package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.PIDController;
import org.firstinspires.ftc.teamcode.auto.Ports;
import org.firstinspires.ftc.teamcode.auto.System;

import java.util.Arrays;


@TeleOp(name = "Telekinetic Operation")
@Config
public class Teleop extends LinearOpMode {

    Ports ports;
    Ports.Builder builder;

    Gamepad prevGamepad1;
    Gamepad prevGamepad2;
    Gamepad currGamepad1;
    Gamepad currGamepad2;

    public static double servoRangeTime = 1;

    boolean intakeInverse = false;
    int handoffStep = 0;

    boolean buttonPressed = false;

    ElapsedTime elapsedTime = new ElapsedTime();

    PIDController lsv_lController;
    PIDController lsv_rController;

    PIDController lsh_lController;
    PIDController lsh_rController;

    public static Position startingPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public static YawPitchRollAngles startingRotation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);


    double max;
    double tolerance = 30;

    // STATE MACHINE VARIABLES
    private String vSlideState = "MANUAL";
    private double lsv_lPower= 0;
    private double lsv_rPower = 0;

    boolean running = true;

    int lsv_lLast;

    int counter;

    Action handoff;

    @Override
    public void runOpMode() {

        builder = new Ports.Builder();
        builder.wheelsActive = true;
        builder.slidesActive = true;
        builder.servosActive = true;
        ports = new Ports(this, builder);

        ports.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.lsv_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.lsv_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.lsh_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ports.lsh_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ports.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.lsv_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.lsv_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.lsh_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ports.lsh_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lsv_lController = new PIDController(0.0127, 0.0004, 0.000001, 0.06, 20, ports.lsv_l);
        lsv_rController = new PIDController(0.0127, 0.0004, 0.000001, 0.06, 20, ports.lsv_r);

        lsh_lController = new PIDController(0.0127, 0.0004, 0.000001, 0, 20, ports.lsh_l);
        lsh_rController = new PIDController(0.0127, 0.0004, 0.000001, 0, 20, ports.lsh_r);

        prevGamepad1 = new Gamepad();
        prevGamepad2 = new Gamepad();
        currGamepad1 = new Gamepad();
        currGamepad2 = new Gamepad();

        MecanumDrive driver = new MecanumDrive(hardwareMap, new Pose2d(1, 1, 1));

        System.Arm arm = new System.Arm(this, true);
        System.Slides slides = new System.Slides(this, true);
        System.Intake intake = new System.Intake(this, true);

        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                MecanumDrive.kinematics.new WheelVelConstraint(50),
                new AngularVelConstraint(3.14)
        ));
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-110, 110);

        Action awesome = new ParallelAction(
                arm.closeSpecimen(),
                new SequentialAction(
                        new SleepAction(0.5),
                        new ParallelAction(
                                slides.raiseSlides(),
                                driver.actionBuilder(driver.localizer.getPose())
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(-2, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                        .build()))
        );

        Action awesomer = new ParallelAction(
                driver.actionBuilder(driver.localizer.getPose())
                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90), velConstraint, accelConstraint)
                        .build(),
                new SequentialAction(
                        slides.lowerSlides(),
                        arm.openSpecimen()
                )
        );

        ElapsedTime handoffElapsedTime = new ElapsedTime();

        boolean driveToHang = false;
        boolean hangDriveBack = false;
        boolean awesomeness = false;

        boolean specimenClosed = false;

        waitForStart();

        elapsedTime.reset();

        ports.intakeRoll.setPosition(0.5);

        while (running) {
            driver.localizer.update();

            prevGamepad1.copy(currGamepad1);
            prevGamepad2.copy(currGamepad2);

            currGamepad1.copy(gamepad1);
            currGamepad2.copy(gamepad2);

            // **** VERTICAL SLIDES ****
            // USE CONTROLLER TO SET STATE
            if(currGamepad2.left_stick_y >= 0.05){
                vSlideState = "MANUAL";
            } else if(currGamepad1.a && !prevGamepad1.a){
                vSlideState = "HIGH CHAMBER";
            } else if(currGamepad1.b && !prevGamepad1.b){
                vSlideState = "HIGH BASKET";
            }

            // USE STATE TO CALCULATE POWER
            if(vSlideState == "MANUAL"){
                double vertical = -currGamepad2.left_stick_y;

                //if(ports.lsv_r.getCurrentPosition() < 4330 && ports.lsv_l.getCurrentPosition() < 4330 || vertical < 0) {
                if(ports.lsv_l.getCurrentPosition() < 4330 || vertical < 0) {
                    lsv_lPower = vertical + 0.06;
                    lsv_rPower = vertical + 0.06;
                } else {
                    lsv_lPower = 0.06;
                    lsv_rPower = 0.06;
                }
            } else if(vSlideState == "HIGH CHAMBER"){
                lsv_lPower = lsv_lController.evaluate(500 - ports.lsv_l.getCurrentPosition());
                //lsv_rPower = lsv_rController.evaluate(1230 - ports.lsv_r.getCurrentPosition());
                lsv_rPower = lsv_rController.evaluate(500 - ports.lsv_l.getCurrentPosition());
            } else if(vSlideState == "HIGH BASKET"){
                lsv_lPower = lsv_lController.evaluate(3650 - ports.lsv_l.getCurrentPosition());
                //lsv_rPower = lsv_rController.evaluate(3500 - ports.lsv_r.getCurrentPosition());
                lsv_rPower = lsv_rController.evaluate(3650 - ports.lsv_l.getCurrentPosition());
            }

            // SET POWER
            if(!hangDriveBack && !driveToHang) {
                ports.lsv_l.setPower(lsv_lPower);
                ports.lsv_r.setPower(lsv_rPower);
            }

            // **** HORIZONTAL SLIDES ****
            double horizontal = currGamepad2.right_stick_x;

            if(!hangDriveBack && !driveToHang) {
                if ((ports.lsh_r.getCurrentPosition() < 2500 && ports.lsh_l.getCurrentPosition() < 2500) || horizontal < 0) {
                    ports.lsh_r.setPower(horizontal);
                    ports.lsh_l.setPower(horizontal);
                } else {
                    ports.lsh_r.setPower(0);
                    ports.lsh_l.setPower(0);
                }
            }

            // X = SQUARE
            // Y = TRIANGLE
            // A = CROSS
            // B = CIRCLE

            // **** MECANUM WHEELS ****

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double drive = -currGamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double yaw = currGamepad1.right_stick_x;

            if(currGamepad1.dpad_left) {
                strafe = -1;
                drive = 0.2;
            }else if(currGamepad1.dpad_right) {
                strafe = 1;
                drive = 0.2;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double fr = (drive - strafe - yaw) * Math.abs(drive - strafe - yaw);
            double fl = (drive + strafe + yaw) * Math.abs(drive + strafe + yaw);
            double br = (drive + strafe - yaw) * Math.abs(drive + strafe - yaw);
            double bl = (drive - strafe + yaw) * Math.abs(drive - strafe + yaw);

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.abs(bl)), Math.abs(br));

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            if(currGamepad2.y && !prevGamepad2.y) {
                intakeInverse = !intakeInverse;
            }

            if(!hangDriveBack && !driveToHang) {
                ports.fl.setPower(fl);
                ports.fr.setPower(fr);
                ports.bl.setPower(bl);
                ports.br.setPower(br);
            }

            // **** FINAL HANG ****
            if(currGamepad2.left_stick_button && currGamepad2.right_trigger>0.8){
                ElapsedTime endTime = new ElapsedTime();
                while(endTime.seconds()<10 && !(currGamepad1.left_stick_button || currGamepad1.right_stick_button)){
                    ports.fr.setPower(0);
                    ports.fl.setPower(0);
                    ports.br.setPower(0);
                    ports.bl.setPower(0);
                    ports.lsh_l.setPower(-0.1);
                    ports.lsh_l.setPower(-0.1);

                    ports.lsv_l.setPower(-1);
                    ports.lsv_r.setPower(-1);
                }
            }

            // **** SERVOS ****
            // flip outtake claw from inside to outside robot
            if(currGamepad2.dpad_up){
                ports.outtakePitchLL.setPosition(0);
                ports.outtakePitchLR.setPosition(1);
                ports.outtakePitchRR.setPosition(1);
                ports.outtakePitchRL.setPosition(0);
                specimenClosed = false;
            } else if(currGamepad2.dpad_down){
                ports.outtakePitchLL.setPosition(1);
                ports.outtakePitchLR.setPosition(0);
                ports.outtakePitchRR.setPosition(0);
                ports.outtakePitchRL.setPosition(1);
                specimenClosed = false;
            }
            // opens and closes outtake claw
            if(currGamepad2.dpad_right){
                ports.outtakeClaw.setPosition(0.25);
            } else if(currGamepad2.dpad_left){
                ports.outtakeClaw.setPosition(0);
            }
            // open and closes intake claw
            if(currGamepad2.x && !prevGamepad2.x){
                if(intakeInverse) {
                    ports.intakeClaw.setPosition(1);
                    intakeInverse = false;
                } else {
                    ports.intakeClaw.setPosition(0);
                }
            }
            // rotates intake claw up and down
            if(currGamepad2.a && !prevGamepad2.a){
                if(intakeInverse){
                    ports.intakePitch.setPosition(0.66);
                    ports.intakeRoll.setPosition(0.5);
                    intakeInverse = false;
                } else {
                    ports.intakePitch.setPosition(0.25);
                    ports.intakeRoll.setPosition(0.5);
                }
            }

            // horizontal rotates intake claw
            if(currGamepad2.b) {
                if(intakeInverse){
                    ports.intakeRoll.setPosition(0.5);
                    intakeInverse = false;
                } else {
                    ports.intakeRoll.setPosition(0.83);
                }
            }

            // specimen claw open and close
            if(currGamepad2.right_bumper && !prevGamepad2.right_bumper){
                ports.specimenClaw.setPosition(0.8);
                handoffElapsedTime.reset();
                ports.outtakePitchLL.setPosition(0.95);
                ports.outtakePitchLR.setPosition(0.05);
                ports.outtakePitchRR.setPosition(0.05);
                ports.outtakePitchRL.setPosition(0.95);
                specimenClosed = true;
            }
            if(handoffElapsedTime.seconds()>1 && specimenClosed) {
                ports.outtakePitchLL.setPosition(0.9);
                ports.outtakePitchLR.setPosition(0.1);
                ports.outtakePitchRR.setPosition(0.1);
                ports.outtakePitchRL.setPosition(0.9);
            }
            if(currGamepad2.left_bumper && !prevGamepad2.left_bumper){
                ports.specimenClaw.setPosition(0);
                ports.outtakePitchLL.setPosition(1);
                ports.outtakePitchLR.setPosition(0);
                ports.outtakePitchRR.setPosition(0);
                ports.outtakePitchRL.setPosition(1);
                specimenClosed = false;
            }

            // **** AUTOMATED HANDOFF ****

            if(currGamepad1.dpad_up && !prevGamepad1.dpad_up && handoffStep == 0){
                handoffStep = 1;
                handoff = new SequentialAction(
                        intake.raiseClaw(),
                        intake.squareIntake(),
                        intake.loosenIntake(),
                        arm.openOuttake(),
                        arm.partial(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                slides.lowerSlides(),
                                slides.retractSlides()
                        ),
                        arm.closeOuttake(),
                        new SleepAction(0.6),
                        intake.openIntake(),
                        arm.rumble(gamepad2));
            }else if(currGamepad1.dpad_up && !prevGamepad1.dpad_up && handoffStep != 0){
                handoffStep = 0;
            }

            if(handoffStep == 1 && !handoff.run(new TelemetryPacket())){
                handoffStep = 0;
            }

            //Handoff Try 2

            if(currGamepad2.start && !prevGamepad2.start && !hangDriveBack){
                driveToHang = !driveToHang;
                counter += 1;
                driver.localizer.setPose(new Pose2d(-40, 70, Math.toRadians(-90)));
                awesome = new ParallelAction(
                        arm.closeSpecimen(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                new ParallelAction(
                                        slides.raiseSlides(),
                                        driver.actionBuilder(driver.localizer.getPose())
                                                .setTangent(Math.toRadians(-90))
                                                .splineToLinearHeading(new Pose2d(-2, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                                .build()))
                );
            }

            if(currGamepad1.start && !prevGamepad1.start && !driveToHang){
                hangDriveBack = !hangDriveBack;
                awesomeness = false;
                counter = 0;
                awesomer = new ParallelAction(
                        driver.actionBuilder(driver.localizer.getPose())
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-40, 70, Math.toRadians(-91)), Math.toRadians(90), velConstraint, accelConstraint)
                                .build(),
                        new SequentialAction(
                                arm.openSpecimen()
                        )
                );
            }

            if(driveToHang){
                driveToHang = !awesome.run(new TelemetryPacket());
            }

            if(hangDriveBack && !awesomeness){
                awesomeness = !awesomer.run(new TelemetryPacket());
                if(awesomeness){
                    awesome = new ParallelAction(
                            arm.closeSpecimen(),
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    new ParallelAction(
                                            slides.raiseSlides(),
                                            driver.actionBuilder(driver.localizer.getPose())
                                                    .setTangent(Math.toRadians(-90))
                                                    .splineToLinearHeading(new Pose2d(-2+counter, 33, Math.toRadians(90)), Math.toRadians(-90), velConstraint, accelConstraint)
                                                    .build()))
                    );
                }
            }

            if(hangDriveBack && awesomeness){
                awesomeness = awesome.run(new TelemetryPacket());
                hangDriveBack = awesomeness;
            }

            //TELEMETRY
            telemetry.addLine("Intake Inverse: " + intakeInverse);
            telemetry.addData("Front Left Pow", ports.fl.getPower());
            telemetry.addData("Back Left Pow", ports.bl.getPower());
            telemetry.addData("Front Right Pow", ports.fr.getPower());
            telemetry.addData("Back Right Pow", ports.br.getPower());
            telemetry.addData("Intake Claw Position", ports.intakeClaw.getPosition());
            telemetry.addData("Outtake Claw Position", ports.outtakeClaw.getPosition());
            telemetry.addData("Linear Slide Vertical Right Position", ports.lsv_r.getCurrentPosition());
            telemetry.addData("Linear Slide Vertical Left Position", ports.lsv_l.getCurrentPosition());
            telemetry.addData("Linear Slide Horizontal Right Position", ports.lsh_r.getCurrentPosition());
            telemetry.addData("Linear Slide Horizontal Left Position", ports.lsh_l.getCurrentPosition());
            telemetry.addData("Handoff step: ", handoffStep);
            telemetry.addData("timer", handoffElapsedTime);
            telemetry.update();

            elapsedTime.reset();
        }

    }
}