package org.firstinspires.ftc.teamcode.common;

import org.checkerframework.checker.units.qual.C;

public class Constants {

    //Drive Train Constants
    public double CLICKS_PER_REV = 384.5;
    public double DIAMETER_OF_WHEEL = 96 / 25.4; //unit is in inches
    public double MAX_VELOCITY_DT = 2700; // unit is clicks/sec
    public double TOLERANCE = 3; //number of clicks or degrees the robot can be off by

    //Swerve
    public double ROT_WHEEL_PER_ROT_INPUT_SHAFT = 16.0/17.0; //1 rotation in opposite directions = 16/17 rotations of wheel
    public double INCHES_PER_ROT_INPUT_SHAFT = (Math.PI * DIAMETER_OF_WHEEL) * ROT_WHEEL_PER_ROT_INPUT_SHAFT; //11.175 inches per rotation of input shaft

    public double CLICKS_PER_INCH = CLICKS_PER_REV / INCHES_PER_ROT_INPUT_SHAFT; //34.406 clicks of top & bottom gears in opposite directions = 1 inch
    public double INCHES_PER_CLICK = 1.0 / CLICKS_PER_INCH;
    public double DEGREES_PER_CLICK = 360.0 / CLICKS_PER_REV;
    public double CLICKS_PER_DEGREE = 1.0/DEGREES_PER_CLICK;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Distance Between swerve module and Center
    public double DISTANCE_BETWEEN_MODULE_AND_CENTER;
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

}
