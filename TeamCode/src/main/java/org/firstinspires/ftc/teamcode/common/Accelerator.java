package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.kinematics.drive.SimplifiedKinematics;

public class Accelerator {
    ElapsedTime accelerationTimer;
    private boolean isAccelerateCycle = false;

    SimplifiedKinematics.DriveType currentDriveType = SimplifiedKinematics.DriveType.NOT_INITIALIZED;
    SimplifiedKinematics.DriveType prevDriveType = SimplifiedKinematics.DriveType.NOT_INITIALIZED;

    public Accelerator(){
        accelerationTimer = new ElapsedTime();
    }

    public double update(double power, SimplifiedKinematics.DriveType dType){
        prevDriveType = currentDriveType;
        currentDriveType = dType;

        if (power == 0) {
            isAccelerateCycle = false;
            return 0.0;
        }

        if (!isAccelerateCycle || prevDriveType != currentDriveType){
            accelerationTimer.reset();
            isAccelerateCycle = true;
        }

        double accelerationFactor = Math.pow(accelerationTimer.seconds()/8.0,3);
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;


    }
}
