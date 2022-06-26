package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPID {
    private double kp, ki, kd; //Maybe: create static variables for the gains, and fine-tune each value

    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;

    public TurnPID(double targetAngle, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetAngle = targetAngle;
        timer.reset();
    }

    public double update(double angleTurned){
        //proportion
        double error = targetAngle - angleTurned;
        error %= 360;   //ensures that error is between [-360, 360]
        error += 360;   //ensures that error is positive
        error %= 360;   //ensures that the error is still within [-360, 360], even after adding 360
        if (error > 180) error -= 360; //if error goes over 180, it should go to -179. Keeps error between -179~180.


        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError + kd * slope) * 0.9 + (0.1 * Math.signum(error));
        //multiply by 0.9 because robot is heavy (heavy + friction = wheels slide while turning = inaccurate). The 0.9 somewhat compensates for that
        //0.1 * Math.signum(error) gives the robot a little kick towards the direction of the error

        //alternative: motorPower =  Math.tanh(kp * error + ki * accumulatedError + kd * slope);

        return motorPower;
    }
}
