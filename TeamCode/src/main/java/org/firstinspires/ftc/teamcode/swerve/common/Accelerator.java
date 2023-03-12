package org.firstinspires.ftc.teamcode.swerve.common;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;


public class Accelerator {
    ElapsedTime accelerationTimer;
    Constants constants = new Constants();
    public boolean actuallyAccelerate = false; //public for telemetry purposes
    public double accelerationFactor = 1; //for telemetry purposes
    private double accelTime = constants.accelTime; //the higher this number is, the longer it takes to send 100% of the calculated power.

    public Accelerator(){
        accelerationTimer = new ElapsedTime();
    }

    public void actuallyAccelerate(boolean accelerate){
        this.actuallyAccelerate = accelerate;
    }

    public void setAccelFactor(double factor){
        accelTime = factor;
    }

    public double getAccelerationFactor(double value, double turnAmount){ //teleop
        double accelerationFactor = 1;

        if (value == 0) {
            accelerationTimer.reset();
            return 0.0;
        }

        if (actuallyAccelerate && Math.abs(turnAmount) < 12) {
            accelerationFactor = (Math.pow(accelerationTimer.seconds(), 0.5) / accelTime) + 0.3;
            if (accelerationFactor > 1) accelerationFactor = 1;
            else if (accelerationFactor < -1) accelerationFactor = -1;
        }

        this.accelerationFactor = accelerationFactor;
        return accelerationFactor;
    }

    public double getAccelerationFactor(){ //auto
        double accelerationFactor = 1;
        if (actuallyAccelerate) {
            accelerationFactor = (Math.pow(accelerationTimer.seconds(), 0.5) / accelTime) + 0.3;
            if (accelerationFactor > 1) accelerationFactor = 1;
            else if (accelerationFactor < -1) accelerationFactor = -1;
        }

        this.accelerationFactor = accelerationFactor;
        return accelerationFactor;
    }

    public void resetAccelerationAuto(){
        accelerationTimer.reset();
    }
}
