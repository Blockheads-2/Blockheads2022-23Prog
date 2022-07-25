package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.IntIterator;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import java.util.HashMap;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class GlobalPosSystem {

    Constants constants = new Constants();

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private int translationalClicks;
    private int rotationalClicks;


    HardwareDrive robot; //this HardwareDrive robot is a completely separate robot to the ones that are initialized in basedrive and autohub ?!??!?

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
        int top = (clicks0 + clicks2)/2;
        int bottom = (clicks1 + clicks3)/2;
        translationalClicks = (int)(top - bottom) / 2;
        rotationalClicks = top - translationalClicks;

        double wheelOrientation = positionArr[2] * constants.DEGREES_PER_CLICK;
        wheelOrientation = Math.toRadians(wheelOrientation);


        if (translationalClicks == 0){
            update(translationalClicks * Math.cos(wheelOrientation), translationalClicks * Math.sin(wheelOrientation) , rotationalClicks, 0);
        }
        else{
            update(translationalClicks * Math.cos(wheelOrientation), translationalClicks * Math.sin(wheelOrientation) ,0, rotationalClicks);
        }
    }


    public void update ( double x, double y, double wheelR, double robotR){
        //update
        positionArr[0] += x * constants.INCHES_PER_CLICK;
        positionArr[1] += y * constants.INCHES_PER_CLICK;
        positionArr[2] += wheelR * constants.DEGREES_PER_CLICK;
        positionArr[3] += robotR * constants.DEGREES_PER_CLICK;

        try {
            FileWriter myWriter = new FileWriter("gpsLog.txt");
            myWriter.write("GPS Log\n");
            myWriter.write((int) positionArr[0] + "\n");
            myWriter.write((int) positionArr[1] + "\n");
            myWriter.write((int) positionArr[2] + "\n");
            myWriter.write((int) positionArr[3] + "\n\n");
            myWriter.close();
        } catch (IOException e) {
           // System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    public void hardResetGPS(){
        //Reset GPS
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

        //Reset Motor Clicks
        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, 0);
        }
    }


    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
        for (int i = 0; i < 3; i++){
            clicks[i] = robot.dtMotors[i].getCurrentPosition();
        }
        return clicks;
    }

    public HashMap<DcMotorEx, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}