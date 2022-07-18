package org.firstinspires.ftc.teamcode.common;

import org.checkerframework.checker.units.qual.C;

public class Constants {

    //Drive Train Constants
    public double CLICKS_PER_REV = 384.5;
    public double DIAMETER_OF_WHEEL = 96 / 25.4; //unit is in inches
    public double MAX_VELOCITY_DT = 2700; // unit is clicks/sec

    //Swerve
    public double CLICKS_PER_INCH = (384.5) / (Math.PI * DIAMETER_OF_WHEEL); //32.382 clicks per inch
    public double INCHES_PER_CLICK = 1.0 / CLICKS_PER_INCH;
    public double DEGREES_PER_CLICK = 360.0 / CLICKS_PER_REV;
    public double CLICKS_PER_DEGREE = 1.0/DEGREES_PER_CLICK;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

}
