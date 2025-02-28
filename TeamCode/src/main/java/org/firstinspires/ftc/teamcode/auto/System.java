package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class System {



    public static class Arm {

        Ports ports;
        Ports.Builder portsBuilder;
        boolean isSpeciSide;

        public Arm(LinearOpMode opMode, boolean isSpeciSide) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
            this.isSpeciSide = isSpeciSide;
        }

        public class CloseSpecimen implements Action {
            ElapsedTime elapsedTime = new ElapsedTime();
            Boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(!initialized) {
                    ports.specimenClaw.setPosition(1);
                }
                if(elapsedTime.seconds()>1) {
                    ports.outtakePitchLL.setPosition(0.9);
                    ports.outtakePitchLR.setPosition(0.1);
                    ports.outtakePitchRR.setPosition(0.1);
                    ports.outtakePitchRL.setPosition(0.9);
                    return false;
                }
                return true;
            }

        }

        public class OpenSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.specimenClaw.setPosition(0);
                ports.outtakePitchLL.setPosition(1);
                ports.outtakePitchLR.setPosition(0);
                ports.outtakePitchRR.setPosition(0);
                ports.outtakePitchRL.setPosition(1);
                return false;
            }

        }

        public class LowerArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.outtakePitchLL.setPosition(1);
                ports.outtakePitchLR.setPosition(0);
                ports.outtakePitchRR.setPosition(0);
                ports.outtakePitchRL.setPosition(1);
                return false;
            }
        }

        public class RaiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.outtakePitchLL.setPosition(0);
                ports.outtakePitchLR.setPosition(1);
                ports.outtakePitchRR.setPosition(1);
                ports.outtakePitchRL.setPosition(0);
                return false;
            }
        }

        public class OpenOuttake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.outtakeClaw.setPosition(0);
                return false;
            }
        }

        public class CloseOuttake implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.outtakeClaw.setPosition(1);
                return false;
            }
        }

        public Action closeSpecimen() {
            return new CloseSpecimen();
        }

        public Action openSpecimen() {
            return new OpenSpecimen();
        }

        public Action raiseArm() {
            return new RaiseArm();
        }

        public Action lowerArm() {
            return new LowerArm();
        }

        public Action openOuttake() {
            return new OpenOuttake();
        }

        public Action closeOuttake() {
            return new CloseOuttake();
        }

    }

    public static class Intake {

        Ports ports;
        Ports.Builder portsBuilder;
        boolean isSpeciSide;

        public Intake(LinearOpMode opMode, boolean isSpeciSide) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
            this.isSpeciSide = isSpeciSide;
        }

        public class LowerPaddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakePitch.setPosition(0);
                return false;
            }
        }

        public class LowerClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakePitch.setPosition(0.25);
                return false;
            }
        }

        public class RaiseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakePitch.setPosition(0.66);
                return false;
            }
        }

        public class OpenIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(0);
                return false;
            }
        }

        public class CloseIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(1);
                return false;
            }
        }

        public class LoosenIntake implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(0.78);
                return false;
            }
        }

        public Action lowerPaddle(){
            return new LowerPaddle();
        }

        public Action lowerClaw(){
            return new LowerClaw();
        }

        public Action raiseClaw(){
            return new RaiseClaw();
        }

        public Action openIntake(){
            return new OpenIntake();
        }

        public Action closeIntake(){
            return new CloseIntake();
        }

        public Action loosenIntake(){
            return new LoosenIntake();
        }

    }

    public static class Slides {

        Ports ports;
        Ports.Builder portsBuilder;
        boolean isSpeciSide;

        PIDController lsv_lController;
        PIDController lsv_rController;
        PIDController lsh_lController;
        PIDController lsh_rController;

        public Slides(LinearOpMode opMode, boolean isSpeciSide) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
            this.isSpeciSide = isSpeciSide;
            ports.lsv_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ports.lsv_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ports.lsh_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ports.lsh_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ports.lsv_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ports.lsv_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ports.lsh_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ports.lsh_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lsv_lController = new PIDController(0.014, 0.0004, 0.000001, 0.06, 30, ports.lsv_l);
            lsv_rController = new PIDController(0.014, 0.0004, 0.000001, 0.06, 30, ports.lsv_r);
            lsh_lController = new PIDController(0.014, 0.0004, 0.000001, 0, 20, ports.lsh_l);
            lsh_rController = new PIDController(0.014, 0.0004, 0.000001, 0, 20, ports.lsh_r);
        }

        public class LowerSlides implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    lsv_lController.setup(-ports.lsv_l.getCurrentPosition());
                    lsv_rController.setup(-ports.lsv_l.getCurrentPosition());
                }

                ports.lsv_l.setPower(lsv_lController.evaluate(-ports.lsv_l.getCurrentPosition()));
                ports.lsv_r.setPower(lsv_rController.evaluate(-ports.lsv_l.getCurrentPosition()));

                packet.addLine("lsv_l pos:" + ports.lsv_l.getCurrentPosition());

                if(ports.lsv_l.getCurrentPosition() < lsv_lController.tolerance){
                    ports.lsv_l.setPower(lsv_lController.Kf);
                    ports.lsv_r.setPower(lsv_rController.Kf);
                }

                return ports.lsv_l.getCurrentPosition() >= lsv_lController.tolerance;
            }
        }

        public class RaiseSlides implements Action{
            boolean initialized = false;
            int target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    if(isSpeciSide){target=2300;}else{target=3200;}
                    lsv_lController.setup(target-ports.lsv_l.getCurrentPosition());
                    lsv_rController.setup(target-ports.lsv_l.getCurrentPosition());
                }

                ports.lsv_l.setPower(lsv_lController.evaluate(target-ports.lsv_l.getCurrentPosition()));
                ports.lsv_r.setPower(lsv_rController.evaluate(target-ports.lsv_l.getCurrentPosition()));

                packet.addLine("lsv_l pos:" + ports.lsv_l.getCurrentPosition());

                if(Math.abs(target-ports.lsv_l.getCurrentPosition()) < lsv_lController.tolerance){
                    ports.lsv_l.setPower(lsv_lController.Kf);
                    ports.lsv_r.setPower(lsv_rController.Kf);
                    return false;
                }
                else{
                    return true;
                }
            }
        }

        public class RetractSlides implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    lsh_lController.setup(-ports.lsh_l.getCurrentPosition());
                    lsh_rController.setup(-ports.lsh_l.getCurrentPosition());
                }

                ports.lsh_l.setPower(lsh_lController.evaluate(-ports.lsh_l.getCurrentPosition()));
                ports.lsh_r.setPower(lsh_rController.evaluate(-ports.lsh_l.getCurrentPosition()));

                packet.addLine("lsh_l pos:" + ports.lsh_l.getCurrentPosition());

                if(ports.lsh_l.getCurrentPosition() < lsh_lController.tolerance){
                    ports.lsh_l.setPower(0);
                    ports.lsh_r.setPower(0);
                    return false;
                }
                else {
                    return true;
                }
            }
        }

        public class ExtendSlides implements Action{
            boolean initialized = false;
            int target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    if(isSpeciSide){target=2100;}else{target=1300;}
                    lsh_lController.setup(target-ports.lsh_l.getCurrentPosition());
                    lsh_rController.setup(target-ports.lsh_l.getCurrentPosition());
                }

                ports.lsh_l.setPower(lsh_lController.evaluate(target-ports.lsh_l.getCurrentPosition()));
                ports.lsh_r.setPower(lsh_rController.evaluate(target-ports.lsh_l.getCurrentPosition()));

                packet.addLine("lsh_l pos:" + ports.lsh_l.getCurrentPosition());

                if(Math.abs(target-ports.lsh_l.getCurrentPosition()) < lsh_lController.tolerance){
                    ports.lsh_l.setPower(0);
                    ports.lsh_r.setPower(0);
                }

                return Math.abs(target-ports.lsh_l.getCurrentPosition()) >= lsh_lController.tolerance;
            }
        }

        public Action lowerSlides() {
            return new LowerSlides();
        }

        public Action raiseSlides() {
            return new RaiseSlides();
        }

        public Action retractSlides() {
            return new RetractSlides();
        }

        public Action extendSlides() {
            return new ExtendSlides();
        }

    }

}
