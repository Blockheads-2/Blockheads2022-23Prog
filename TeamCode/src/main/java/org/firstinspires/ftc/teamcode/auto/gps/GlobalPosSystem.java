package org.firstinspires.ftc.teamcode.auto.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.IntIterator;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import java.util.HashMap;

public class GlobalPosSystem {

    Constants constants = new Constants();

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private int translationalClicks;
    private int rotationalClicks;




    HardwareDrive robot;

    public GlobalPosSystem(){
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

    }

    public void calculatePos(){
        int clicks0;
        int clicks1;
        int clicks2;
        int clicks3;

        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition());
        }

        //Left
        clicks0 = motorClicksPose.get(robot.dtMotors[0]); //A
        clicks1 = motorClicksPose.get(robot.dtMotors[1]); //B

        //Right
        clicks2 = motorClicksPose.get(robot.dtMotors[2]); //A
        clicks3 = motorClicksPose.get(robot.dtMotors[3]); //B

        //distance = (A-B)/2
        translationalClicks = ((clicks0 + clicks2)/2 - (clicks1 + clicks3)/2);
        translationalClicks /= 2;

        rotationalClicks = (clicks0 + clicks2)/2 - translationalClicks;

        double w = positionArr[2] * constants.DEGREES_PER_CLICK;
         w = Math.toRadians(w);


        if (translationalClicks == 0){
            update(translationalClicks * Math.cos(w), translationalClicks * Math.sin(w) , rotationalClicks, 0);
        }
        else{
            update(translationalClicks * Math.cos(w), translationalClicks * Math.sin(w) ,0, rotationalClicks);
        }

    }

    public void update(double x, double y, double wheelR, double robotR){
        //update
        positionArr[0] = x * constants.INCHES_PER_CLICK;
        positionArr[1] = y * constants.INCHES_PER_CLICK;
        positionArr[2] = wheelR * constants.DEGREES_PER_CLICK;
        positionArr[3] = robotR * constants.DEGREES_PER_CLICK;

    }

    public double[] getPositionArr(){
        return positionArr;
    }

    public HashMap<DcMotorEx, Integer> getMotorClicksPose(){
        return motorClicksPose;
    }


}
