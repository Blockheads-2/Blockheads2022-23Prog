package org.firstinspires.ftc.teamcode.auto.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.IntIterator;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import java.util.HashMap;

public class GlobalPosSystem {

    private double[] position = new double[3];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();

    HardwareDrive robot;

    public GlobalPosSystem(){
        for (int i = 0; i < 3; i++){
            position[i] = 0;
        }

    }

    public void calculatePos(){
        int translationalClicks;
        int rotationalClicks;

        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition());
        }

        // 0 and 1 are left
        // 2 and 3 are right



    }

    public void update(double x, double y, double theta){
        //update
        position[0] = x;
        position[1] = y;
        position[2] = theta;
    }

    public double[] getPosition(){
        return position;
    }

    public HashMap<DcMotorEx, Integer> getMotorClicksPose(){
        return motorClicksPose;
    }


}
