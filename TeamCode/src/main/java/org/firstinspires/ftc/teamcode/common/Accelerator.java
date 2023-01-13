package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;


public class Accelerator {
    ElapsedTime accelerationTimer;
    Constants constants = new Constants();
    private boolean isAccelerateCycle = false;

    public Accelerator(){
        accelerationTimer = new ElapsedTime();
    }


    public double update(double power, boolean actuallyAccelerate){
        if (power == 0) {
            accelerationTimer.reset();
            return 0.0;
        }

        double accelerationFactor = (Math.pow(accelerationTimer.seconds(), 0.5)/ constants.accelTime) + 0.3;
        power *= accelerationFactor;

        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        return power;
    }

    public double update(double power){ //this gets rid of the accelerator.
        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
    }
}
