package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * USAGE GUIDE:
 *
 * Call setup with the current position and then in a loop call evaluate until you reach your
 * desired outcome.
 */

public class PIDController {

    //Define the parameters of the PID controller
    double Kp;
    double Ki;
    double Kd;
    double Kf;
    int tolerance;
    int lastError;
    double integralSum;
    ElapsedTime elapsedTime;
    DcMotor slide;
    boolean isSetup = false;


    public PIDController(double Kp, double Ki, double Kd, double Kf, int tolerance, DcMotor output){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        elapsedTime = new ElapsedTime();
        lastError = 0;
        integralSum = 0;
        slide = output;
        this.tolerance = tolerance;
    }


    public void setup(int error){
        lastError = error;
        elapsedTime.reset();
    }

    public double evaluate(int error){
        integralSum += error * elapsedTime.time();
        double output = error * Kp + integralSum * Ki + (error - lastError) * Kd / elapsedTime.time() + Kf;
        elapsedTime.reset();
        return (output);

    }

    public void runTo(int target){
        if(!isSetup){
            setup(target-slide.getCurrentPosition());
            isSetup = true;
        }

        while (slide.getCurrentPosition() > target + tolerance || slide.getCurrentPosition() < target - tolerance) {
            slide.setPower(evaluate(target - slide.getCurrentPosition()));
        }

        slide.setPower(0);
        isSetup = false;
    }

    public DcMotor getSlide() {
        return slide;
    }
}
