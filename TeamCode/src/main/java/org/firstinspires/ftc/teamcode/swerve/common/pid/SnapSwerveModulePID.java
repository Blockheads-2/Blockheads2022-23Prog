package org.firstinspires.ftc.teamcode.swerve.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;

import java.util.logging.Level;
import java.util.logging.Logger;

public class SnapSwerveModulePID {
    private double kp, ki, kd;
    private double pError;

    private ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;


    private final static Logger LOGGER =
            Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public SnapSwerveModulePID(){
        timer.reset();
    }

    double error = 0;
    public double update(double amountLeft){
        //proportion
//        double error = target - amountLeft;
        error = amountLeft;

        pError = error;

        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError - kd * slope);
//        if (Math.abs(motorPower) <= 0.05) motorPower = 0;

        return motorPower;
    }

    public void setTargets(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        resetValues();
    }

    public void resetValues(){
        timer.reset();
        prevError = 0;
        prevTime = 0;
        accumulatedError = 0;
        target = 0;
    }

    public double getError(){
        return error;
    }

    public void makeSomeLog() {
        // add some code of your choice here
        // Moving to the logging part now
        LOGGER.log(Level.INFO, "Error: " + pError);

    }
}
