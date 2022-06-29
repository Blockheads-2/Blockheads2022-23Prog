package org.firstinspires.ftc.teamcode.auto.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import java.util.HashMap;

public class GlobalPosSystem {

    private final LinearOpMode linearOpMode;
    private double[] position = new double[3];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<DcMotorEx, Integer>();

    HardwareDrive robot;

    public GlobalPosSystem(LinearOpMode plinear){
        for (int i = 0; i < 3; i++){
            position[i] = 0;
        }

        linearOpMode = plinear;

    }

    public void calculatePos(){
        int translationalClicks;
        int rotationalClicks;

        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition());

        }





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


}
