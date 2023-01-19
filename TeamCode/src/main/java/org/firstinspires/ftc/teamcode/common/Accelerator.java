package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;


public class Accelerator {
    ElapsedTime accelerationTimer;
    Constants constants = new Constants();
    private boolean actuallyAccelerate = false;
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


    public double update(double power, double turnAmountLeft) {
        if (actuallyAccelerate && Math.abs(turnAmountLeft) > constants.degreeTOLERANCE) {
            if (power == 0) {
                accelerationTimer.reset();
                return 0.0;
            }

            double accelerationFactor = (Math.pow(accelerationTimer.seconds(), 0.5) / accelTime) + 0.3;
            power *= accelerationFactor;
        }
        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        return power;
    }
}
