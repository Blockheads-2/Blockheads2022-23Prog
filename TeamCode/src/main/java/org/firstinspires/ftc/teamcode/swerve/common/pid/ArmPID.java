package org.firstinspires.ftc.teamcode.swerve.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.logging.Logger;

public class ArmPID {
    private double kp, ki, kd;
    private double pError;

    private ElapsedTime timer = new ElapsedTime();
    private double targetClicks = 0;

    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;

    private final static Logger LOGGER =
            Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public ArmPID(){
        timer.reset();
    }

    double error = 0;
    public double update(double currClicks){
        //proportion
        error = targetClicks - currClicks;

        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError - kd * slope) * 0.4 + (0.1 * Math.signum(error));
        //double motorPower =  Math.tanh(kp * error + ki * accumulatedError + kd * slope);

        return motorPower;
    }

    public double getError(){
        return error;
    }

    public double getTarget(){
        return targetClicks;
    }

    public void setTargets(double targetClicks, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetClicks = targetClicks;
    }
}
