package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * USAGE GUIDE:
 *
 * The "Ports" class is designed to make it easier to access hardware. The first section can be used
 * to declare the hardware variables, without attaching them to anything. The second section
 * (within the init function) is for attaching those variables to their various hardware and running
 * other initialization steps. This is run before the challenge starts, so no need to worry about
 * efficiency.
 */

public class Ports {

    //CREATE HARDWARE VARIABLES HERE

    //Motors
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor br;
    public DcMotor bl;

    // Linear slides
    public DcMotor lsv_r;
    public DcMotor lsv_l;
    public DcMotor lsh_r;
    public DcMotor lsh_l;

    // Intake Servos
    public Servo intakeClaw;
    public Servo intakePitch;
    public Servo intakeRoll;
    public Servo specimenClaw;

    //Outtake Servos
    public Servo outtakeClaw;
    public Servo outtakePitchRL;
    public Servo outtakePitchRR;
    public Servo outtakePitchLL;
    public Servo outtakePitchLR;


    //DO INITIALIZATION STEPS HERE
     public Ports(LinearOpMode opMode, Builder builder) {

         if (builder.allActive || builder.wheelsActive || builder.frActive) {
             fr = opMode.hardwareMap.get(DcMotor.class, "FR");
             fr.setDirection(DcMotor.Direction.REVERSE);
             fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         }

         if (builder.allActive || builder.wheelsActive || builder.flActive) {
             fl = opMode.hardwareMap.get(DcMotor.class, "FL");
             fl.setDirection(DcMotor.Direction.FORWARD);
             fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         }

         if (builder.allActive || builder.wheelsActive || builder.brActive) {
             br = opMode.hardwareMap.get(DcMotor.class, "BR");
             br.setDirection(DcMotor.Direction.REVERSE);
             br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         }

         if (builder.allActive || builder.wheelsActive || builder.blActive) {
             bl = opMode.hardwareMap.get(DcMotor.class, "BL");
             bl.setDirection(DcMotor.Direction.FORWARD);
             bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         }

         if (builder.allActive || builder.slidesActive || builder.lsv_rActive) {
             lsv_r = opMode.hardwareMap.get(DcMotor.class, "lsv_r");
         }

         if (builder.allActive || builder.slidesActive || builder.lsv_lActive) {
             lsv_l = opMode.hardwareMap.get(DcMotor.class, "lsv_l");
             lsv_l.setDirection(DcMotor.Direction.REVERSE);
         }

         if (builder.allActive || builder.slidesActive || builder.lsh_rActive) {
             lsh_r = opMode.hardwareMap.get(DcMotor.class, "lsh_r");
         }

         if (builder.allActive || builder.slidesActive || builder.lsh_lActive) {
             lsh_l = opMode.hardwareMap.get(DcMotor.class, "lsh_l");
             lsh_l.setDirection(DcMotor.Direction.REVERSE);
         }

         if (builder.allActive || builder.servosActive || builder.intakeClawActive) {
             intakeClaw = opMode.hardwareMap.get(Servo.class, "intakeClaw");
         }

         if (builder.allActive || builder.servosActive || builder.intakePitchActive) {
             intakePitch = opMode.hardwareMap.get(Servo.class, "intakePitch");
         }

         if (builder.allActive || builder.servosActive || builder.intakeRollActive) {
             intakeRoll = opMode.hardwareMap.get(Servo.class, "intakeRoll");
         }

         if (builder.allActive || builder.servosActive || builder.intakeClawActive) {
             intakeClaw = opMode.hardwareMap.get(Servo.class, "intakeClaw");
         }

         if (builder.allActive || builder.servosActive || builder.outtakeClawActive) {
             outtakeClaw = opMode.hardwareMap.get(Servo.class, "outtakeClaw");
         }

         if (builder.allActive || builder.servosActive || builder.outtakePitchRLActive) {
             outtakePitchRL = opMode.hardwareMap.get(Servo.class, "outtakePitchRL");
         }

         if (builder.allActive || builder.servosActive || builder.outtakePitchRRActive) {
             outtakePitchRR = opMode.hardwareMap.get(Servo.class, "outtakePitchRR");
         }

         if (builder.allActive || builder.servosActive || builder.outtakePitchLLActive) {
             outtakePitchLL = opMode.hardwareMap.get(Servo.class, "outtakePitchLL");
         }

         if (builder.allActive || builder.servosActive || builder.outtakePitchLRActive) {
             outtakePitchLR = opMode.hardwareMap.get(Servo.class, "outtakePitchLR");
         }

         if (builder.allActive || builder.servosActive || builder.specimenClawActive) {
             specimenClaw = opMode.hardwareMap.get(Servo.class, "specimenClaw");
         }
     }

     public static class Builder {
         public boolean wheelsActive = false;
         public boolean slidesActive = false;
         public boolean intakeClawActive = false;
         public boolean intakePitchActive = false;
         public boolean intakeRollActive = false;
         public boolean outtakeClawActive = false;
         public boolean outtakePitchRLActive = false;
         public boolean outtakePitchRRActive = false;
         public boolean outtakePitchLLActive = false;
         public boolean outtakePitchLRActive = false;
         public boolean specimenClawActive = false;
         public boolean allActive = false;
         public boolean frActive = false;
         public boolean flActive = false;
         public boolean brActive = false;
         public boolean blActive = false;
         public boolean lsv_rActive = false;
         public boolean lsv_lActive = false;
         public boolean lsh_rActive = false;
         public boolean lsh_lActive = false;
         public boolean servosActive = false;
     }
}