package org.firstinspires.ftc.teamcode.common.positioning;

import org.firstinspires.ftc.teamcode.common.Constants;

public class MathConstHeadSwerve {
    Constants constants = new Constants();

    //declaring local class variables
    double x = 0;
    double y = 0;
    double distance = 0;
    double theta = 0;

    public void setFinalPosition(double xPosition, double yPosition){
        x = xPosition;
        y = yPosition;
    }

    public double getDistance(){
        distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        return distance;
    }

    public double getTheta(){
        theta = Math.atan2(y,x);
        return theta;
    }

}
