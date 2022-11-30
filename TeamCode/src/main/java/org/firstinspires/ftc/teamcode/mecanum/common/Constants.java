package org.firstinspires.ftc.teamcode.mecanum.common;

public class Constants {

    //Arm Positions

    //Bottom
    public int bottomMotorBottom = 1300;
    public int topMotorBottom = 260;
    public double armMotorBottom = 0.3;

    //Low
    public int bottomMotorLow = 500;
    public int topMotorLow = 350;
    public double armMotorLow = 0.3;

    //Mid
    public int bottomMotorMid = 400;
    public int topMotorMid = 460;
    public double armMotorMid = 0.3;
    
    //High
    public int bottomMotorHigh = 800;
    public int topMotorHigh = 560;
    public int armMotorHigh;

    //Claw Position
    public double initializedArmServo = 0.5;
    public double initializedClaw = 0;

    //Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Drive Train Constants
    public double maxVelocityDT = 2700;
    public double clicksPerInch = 45.285;
}
