package org.firstinspires.ftc.teamcode.mecanum.common;

public class Constants {

    //Arm Positions

    //Bottom
    public int bottomMotorBottom = 1120;
    public int topMotorBottom = 60;
    public double armServoBottom = 0.05;

    //Low
    public int bottomMotorLow = 140;
    public int topMotorLow = 120;
    public double armServoLow = 0.4;

    //Mid
    public int bottomMotorMid = 420;
    public int topMotorMid = 340;
    public double armServoMid = 0.4;
    
    //High
    public int bottomMotorHigh = 600;
    public int topMotorHigh = 600;
    public double armServoHigh = 0.2;

    //Claw Position
    public double clawUp = 0;
    public double clawDown = 0.8;
    public double openClaw = 0.5;
    public double closeClaw = 1;


    //Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Drive Train Constants
    public double maxVelocityDT = 2700;
    public double clicksPerInch = 45.285;
}
