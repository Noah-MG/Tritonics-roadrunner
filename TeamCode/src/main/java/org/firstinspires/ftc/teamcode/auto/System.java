package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class System {



    public static class Arm {

        Ports ports;
        Ports.Builder portsBuilder;

        public Arm(LinearOpMode opMode) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
        }

        public class CloseSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.specimenClaw.setPosition(1);
                return false;
            }

        }

        public class OpenSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.specimenClaw.setPosition(0);
                return false;
            }

        }

        public Action closeSpecimen() {
            return new CloseSpecimen();
        }

        public Action openSpecimen() {
            return new OpenSpecimen();
        }

    }

    public static class Intake {

        Ports ports;
        Ports.Builder portsBuilder;

        public Intake(LinearOpMode opMode) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
        }

        public class LowerPaddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(0);
                return false;
            }
        }

        public class LowerClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(0.4);
                return false;
            }
        }

        public class RaiseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ports.intakeClaw.setPosition(1);
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

    }

    public static class Slides {

        Ports ports;
        Ports.Builder portsBuilder;

        PIDController lsv_lController;
        PIDController lsv_rController;
        PIDController lsh_lController;
        PIDController lsh_rController;

        public Slides(LinearOpMode opMode) {
            portsBuilder = new Ports.Builder();
            portsBuilder.allActive = true;
            ports = new Ports(opMode, portsBuilder);
            lsv_lController = new PIDController(0.0127, 0.0004, 0.000001, 0.1, 20, ports.lsv_l);
            lsv_rController = new PIDController(0.0127, 0.0004, 0.000001, 0.1, 20, ports.lsv_r);
            lsh_lController = new PIDController(0.0127, 0.0004, 0.000001, 0, 20, ports.lsh_l);
            lsh_rController = new PIDController(0.0127, 0.0004, 0.000001, 0, 20, ports.lsh_r);
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

                return ports.lsv_l.getCurrentPosition() >= lsv_lController.tolerance;
            }
        }

        public class RaiseSlides implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    lsv_lController.setup(2300-ports.lsv_l.getCurrentPosition());
                    lsv_rController.setup(2300-ports.lsv_l.getCurrentPosition());
                }

                ports.lsv_l.setPower(lsv_lController.evaluate(2300-ports.lsv_l.getCurrentPosition()));
                ports.lsv_r.setPower(lsv_rController.evaluate(2300-ports.lsv_l.getCurrentPosition()));

                return Math.abs(2300-ports.lsv_l.getCurrentPosition()) >= lsv_lController.tolerance;
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

                return ports.lsh_l.getCurrentPosition() >= lsh_lController.tolerance;
            }
        }

        public class ExtendSlides implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    initialized = true;
                    lsh_lController.setup(2500-ports.lsh_l.getCurrentPosition());
                    lsh_rController.setup(2500-ports.lsh_l.getCurrentPosition());
                }

                ports.lsh_l.setPower(lsh_lController.evaluate(2500-ports.lsh_l.getCurrentPosition()));
                ports.lsh_r.setPower(lsh_rController.evaluate(2500-ports.lsh_l.getCurrentPosition()));

                return Math.abs(2500-ports.lsh_l.getCurrentPosition()) >= lsh_lController.tolerance;
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
