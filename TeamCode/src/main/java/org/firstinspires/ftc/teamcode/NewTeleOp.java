package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.PIDController;
import org.firstinspires.ftc.teamcode.auto.Ports;


@TeleOp(name = "TeleOp State")
@Config
public class NewTeleOp extends LinearOpMode {

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


        ElapsedTime handoffElapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        ports.intakeRoll.setPosition(0.5);

        while (running) {

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
            ports.lsv_l.setPower(lsv_lPower);
            ports.lsv_r.setPower(lsv_rPower);


            // **** HORIZONTAL SLIDES ****
            double horizontal = currGamepad2.right_stick_x;

            if(ports.lsh_r.getCurrentPosition() < 2500 && ports.lsh_l.getCurrentPosition() < 2500) {
                ports.lsh_r.setPower(horizontal);
                ports.lsh_l.setPower(horizontal);
            } else {
                ports.lsh_r.setPower(0);
                ports.lsh_l.setPower(0);
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
                strafe = 1;
                drive = 0.2;
            }else if(currGamepad1.dpad_right) {
                strafe = -1;
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

            ports.fl.setPower(fl);
            ports.fr.setPower(fr);
            ports.bl.setPower(bl);
            ports.br.setPower(br);

            // **** FINAL HANG ****
            if(currGamepad1.left_stick_button && currGamepad2.left_stick_button){
                running = false;
            }

            // **** SERVOS ****
            // flip outtake claw from inside to outside robot
            if(currGamepad2.dpad_up){
                ports.outtakePitchLL.setPosition(0);
                ports.outtakePitchLR.setPosition(1);
                ports.outtakePitchRR.setPosition(1);
                ports.outtakePitchRL.setPosition(0);
            } else if(currGamepad2.dpad_down){
                ports.outtakePitchLL.setPosition(1);
                ports.outtakePitchLR.setPosition(0);
                ports.outtakePitchRR.setPosition(0);
                ports.outtakePitchRL.setPosition(1);
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
                    ports.intakeRoll.setPosition(0.17);
                }
            }

            // specimen claw open and close
            if(currGamepad2.right_bumper && !prevGamepad2.right_bumper){
                ports.specimenClaw.setPosition(0.8);
                ports.outtakePitchLL.setPosition(0.9);
                ports.outtakePitchLR.setPosition(0.1);
                ports.outtakePitchRR.setPosition(0.1);
                ports.outtakePitchRL.setPosition(0.9);
            }
            if(currGamepad2.left_bumper && !prevGamepad2.left_bumper){
                ports.specimenClaw.setPosition(0);
                ports.outtakePitchLL.setPosition(0.95);
                ports.outtakePitchLR.setPosition(0.05);
                ports.outtakePitchRR.setPosition(0.05);
                ports.outtakePitchRL.setPosition(0.95);
            }

            // **** AUTOMATED HANDOFF ****

            if(currGamepad1.dpad_up && !prevGamepad1.dpad_up && handoffStep == 0){
                handoffStep = 1;
                handoffElapsedTime.reset();
            }



            //Handoff Try 2

            if(handoffStep == 1) {
                ports.fr.setPower(0);
                ports.br.setPower(0);
                ports.fl.setPower(0);
                ports.bl.setPower(0);

                ports.intakePitch.setPosition(0.66);
                ports.outtakeClaw.setPosition(0);
                ports.intakeClaw.setPosition(0.78);

                ports.outtakePitchLL.setPosition(0.97);
                ports.outtakePitchLR.setPosition(0.03);
                ports.outtakePitchRR.setPosition(0.03);
                ports.outtakePitchRL.setPosition(0.97);

                lsv_lController.setup(-ports.lsv_l.getCurrentPosition());
                lsv_rController.setup(-ports.lsv_l.getCurrentPosition());
                lsh_lController.setup(-ports.lsh_l.getCurrentPosition());
                lsh_rController.setup(-ports.lsh_r.getCurrentPosition());
                if(handoffElapsedTime.milliseconds() > 300) {
                    handoffStep = 2;
                }
            }

            if(handoffStep == 2) {
                ports.outtakeClaw.setPosition(0);

                ports.lsv_l.setPower(lsv_lController.evaluate(-ports.lsv_l.getCurrentPosition())); // error: between 0 and current pos
                ports.lsv_r.setPower(lsv_rController.evaluate(-ports.lsv_l.getCurrentPosition()));
                ports.lsh_l.setPower(lsh_lController.evaluate(-ports.lsh_l.getCurrentPosition()));
                ports.lsh_r.setPower(lsh_rController.evaluate(-ports.lsh_r.getCurrentPosition()));

                if(ports.lsv_l.getCurrentPosition() < 10 && (Math.abs(-ports.lsh_l.getCurrentPosition()) < 10 || Math.abs(-ports.lsh_r.getCurrentPosition()) < 10)) {
                    handoffStep = 3;
                    handoffElapsedTime.reset();
                }
            }

            if(handoffStep == 3) {
                ports.outtakeClaw.setPosition(1);
                if(handoffElapsedTime.milliseconds() > 1000) {
                    handoffStep = 4;
                }
            }
            // You might want to use a time to add a little
            // time between closing outtake and opening intake
            // just to avoid the possibility of dropping.
            if(handoffStep == 4) {
                ports.intakeClaw.setPosition(0);
                lsh_lController.setup(350-ports.lsh_l.getCurrentPosition());
                lsh_rController.setup(350-ports.lsh_r.getCurrentPosition());
                handoffStep = 5;
            }

            if(handoffStep == 5) {
                ports.lsh_l.setPower(lsh_lController.evaluate(350-ports.lsh_l.getCurrentPosition()));
                ports.lsh_r.setPower(lsh_rController.evaluate(350-ports.lsh_r.getCurrentPosition()));

                if((Math.abs(ports.lsh_l.getCurrentPosition()-350) < 10 || Math.abs(ports.lsh_r.getCurrentPosition()-350) < 10)){
                    handoffStep = 0;
                }
            }

             /*
             * 1. intakeClaw -> 0.05 & outtakeClaw -> 0.15 & intakePitch -> 0.4 & outtakePitch -> 0 & horizontalSlides -> 1700 if horizontalSlides < 1700 & verticalSlides -> 0
             * 2. horizontalSlides -> 910 & intakeClaw -> 0.04
             * 3. outtakeClaw -> 0.25 & intakeClaw -> 0.03
             * 4. horizontalSlides -> 1300
             */


//            if(currGamepad1.dpad_up && !prevGamepad1.dpad_up) {
//                handoffStep = 1;
//                // Why negative? -SC
//                lsv_lController.setup(ports.lsv_l.getCurrentPosition());
//                lsv_rController.setup(ports.lsv_r.getCurrentPosition());
//            }
//            if(handoffStep == 1){
//                // set claw to closed
//                ports.intakeClaw.setPosition(0.85);
//                // set intake claw open
//                ports.outtakeClaw.setPosition(0.25);
//                telemetry.addLine("set intake claw closed, set outtake claw open");
//                telemetry.update();
//                // I would avoid sleep() in teleop.  You have a loop running
//                // and you are interrupting that loop.  Use a timer and a while
//                // loop instead.  THen use state variables like handoffStep to control
//                // what can run or can't run while you are waiting.
//                // bring intake claw up and over
//                ports.intakePitch.setPosition(0.3);
//                // bring outtake claw up and over
//                ports.outtakePitchL.setPosition(0);
//                ports.outtakePitchR.setPosition(1);
//                telemetry.addLine("set intake claw up and over, set outtake claw up and over");
//                telemetry.update();
                // bring slides out enough so that the incoming claw doesn't bash them
//                if(ports.lsh_l.getCurrentPosition() < 1700 || ports.lsh_r.getCurrentPosition() < 1700){
//                    ports.lsh_r.setPower(1);
//                } else {
//                    ports.lsh_l.setPower(0);
//                    ports.lsh_r.setPower(0);
//                }
                // bring linear slides down
//                ports.lsv_l.setPower(lsv_lController.evaluate(ports.lsv_l.getCurrentPosition()));
//                ports.lsv_r.setPower(lsv_rController.evaluate(ports.lsv_r.getCurrentPosition()));
//                // begin handoff setup 2
//                // Shouldn't lsh_l and lsh_r be greater than 1700? -SC
//                if((ports.lsv_l.getCurrentPosition() < 20 || ports.lsv_r.getCurrentPosition() < 20) && (ports.lsh_l.getCurrentPosition() < 1700 || ports.lsh_r.getCurrentPosition() < 1700)){
//                    handoffStep = 2;
//                    telemetry.addLine("here now 1!");
                //}
//            }
//            if(handoffStep == 2){
//                telemetry.addLine("here now 2!");
//                // slightly widen intake claw so it's ready to hand the specimen off to the outtake claw
//                ports.intakeClaw.setPosition(0.04);
//                // bring the horizontal linear slides in
//                ports.lsh_l.setPower(1);
//                ports.lsh_r.setPower(1);
//                // begin step 3
//                if(ports.lsh_l.getCurrentPosition() <  940 || ports.lsh_r.getCurrentPosition() < 940){
//                    ports.lsh_l.setPower(0);
//                    ports.lsh_r.setPower(0);
//                    handoffStep = 3;
//                }
//            }
//            if(handoffStep == 3){
//                // outtake claw grabs onto the specimen
//                ports.outtakeClaw.setPosition(0.25);
//                // intake claw releases specimen
//                ports.intakeClaw.setPosition(0.03);
//                // begin handoff step 4
//                if(ports.intakeClaw.getPosition() < 0.35){
//                    handoffStep = 4;
//                }
//            }
//            if(handoffStep == 4){
//                // bring horizontal linear slides out enough
//                ports.lsh_l.setPower(1);
//                ports.lsh_r.setPower(1);
//                if(ports.lsh_l.getCurrentPosition() > 1700 || ports.lsh_r.getCurrentPosition() > 1700){
//                    ports.lsh_l.setPower(0);
//                    ports.lsh_r.setPower(0);
//                    handoffStep = 0;
//                }
//            }

            // WHAT NEEDS TO BE ADDED TO GAMEPAD 1
            // low basket set position and high basket set position
            // low chamber and high chamber set position
            // move a set distance back from the chamber to lock the specimen on the hanging bar

            // and make a gamepad 2 function for the bumper that widens the outtake claw slightly before the manual handoff to the outtake claw


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
            telemetry.addData("button pressed:((((( ", gamepad1.dpad_up);
            telemetry.addData("timer", handoffElapsedTime);
            telemetry.update();

            elapsedTime.reset();
        }

        ports.fr.setPower(0);
        ports.fl.setPower(0);
        ports.br.setPower(0);
        ports.bl.setPower(0);

        ports.lsv_l.setPower(-1);
        ports.lsv_r.setPower(-1);
        sleep(10000);
        ports.lsv_l.setPower(0);
        ports.lsv_r.setPower(0);
    }
}