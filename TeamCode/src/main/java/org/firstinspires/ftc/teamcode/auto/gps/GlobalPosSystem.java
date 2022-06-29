package org.firstinspires.ftc.teamcode.auto.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

public class GlobalPosSystem {

    private final LinearOpMode linearOpMode;
    private double[] position = new double[3];
    private int[] motorClicksPos = new int[4];

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

        //Top Left; Bottom Left; Top Right; Bottom Right
        motorClicksPos[0] = robot.topL.getCurrentPosition();
        motorClicksPos[1] = robot.botL.getCurrentPosition();
        motorClicksPos[2] = robot.topR.getCurrentPosition();
        motorClicksPos[3] = robot.botR.getCurrentPosition();





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
