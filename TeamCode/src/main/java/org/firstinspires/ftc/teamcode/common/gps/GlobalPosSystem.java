package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.IntIterator;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics;

import java.util.HashMap;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class GlobalPosSystem {

    Constants constants = new Constants();
    Kinematics kinematics;

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private HashMap<DcMotorEx, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public GlobalPosSystem(HardwareDrive robot){
        this.robot = robot;
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }
        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition()); //(key, value)
        }
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void calculatePos(){
        if (!goodGap()) return; //this may or may not be useful
        for (DcMotorEx motors : robot.dtMotors){
            prevMotorClicks.put(motors, motorClicksPose.get(motors)); //(key, value)
            motorClicksPose.put(motors, motors.getCurrentPosition()); //(key, value)
        }

        //Left side
        int topL = motorClicksPose.get(robot.dtMotors[0]) - prevMotorClicks.get(robot.dtMotors[0]); //change in top left
        int bottomL = motorClicksPose.get(robot.dtMotors[1]) - prevMotorClicks.get(robot.dtMotors[1]); //change in bottom left
        int translationalClicksL = (topL - bottomL) / 2; //distance = (top-bottom)/2
        int rotationalClicksL = topL - translationalClicksL;

        //Right side
        int topR = motorClicksPose.get(robot.dtMotors[2]) - prevMotorClicks.get(robot.dtMotors[2]); //change in top right
        int bottomR = motorClicksPose.get(robot.dtMotors[3]) - prevMotorClicks.get(robot.dtMotors[3]); //change in bottom right
        int translationalClicksR = (topR - bottomR) / 2; //distance = (top-bottom)/2
        int rotationalClicksR = topR - translationalClicksR;

        //the rotational clicks for the left and right should always be the same.
        int rotationalClicks = (rotationalClicksL + rotationalClicksR) / 2;

        double wheelOrientation = positionArr[2] * constants.DEGREES_PER_CLICK; //this will be off by 1 loop.
        wheelOrientation = Math.toRadians(wheelOrientation);

        double theta = 0.0;

        if (kinematics.getDriveType() == Kinematics.DriveType.SPLINE){
            double bigArc = Math.max(translationalClicksL, translationalClicksR);
            double smallArc = Math.min(translationalClicksL, translationalClicksR);
            theta = Math.toDegrees((bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER));
        }

        int translationalClicks = (translationalClicksL + translationalClicksR) / 2; //make it an int or double???
        if (translationalClicks == 0){
            update(translationalClicks * Math.cos(wheelOrientation), translationalClicks * Math.sin(wheelOrientation) , rotationalClicks, theta);
            /*
            Problem:
            Calculates incorrect x & y position when wheels are traveling at an arc.
            Because then x != distance * cos(angle)

            To Do:
            - "Clamp" the output of orientations to (-180, 180], to keep it uniform with the rest of the program.
            - Test if math works
             */
        }
        else{
            update(translationalClicks * Math.cos(wheelOrientation), translationalClicks * Math.sin(wheelOrientation) ,0, rotationalClicks + theta);
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

    private boolean goodGap(){
        for (int i = 0; i < 3; i++) {
            try{
                if (Math.abs(motorClicksPose.get(robot.dtMotors[i]) - prevMotorClicks.get(robot.dtMotors[i])) <= constants.TOLERANCE) return false;
            } catch (NullPointerException nullPointerException){
                return false;
            }
        }
        return true;
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