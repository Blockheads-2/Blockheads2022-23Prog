package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class TurnMath {
    Constants constants = new Constants();

    private double theta;
    private double arcLength;

    private int targetClicks;

    public void setPos(double theta, int initClicks, int direction, boolean rightPod){
        this.theta = theta;

        targetClicks = (int)(getDistance() * constants.CLICKS_PER_INCH * direction) + initClicks;

        if (rightPod) targetClicks *= -1;

        this.arcLength = Math.abs(getDistance());
    }

    public double getDistance(){
        double arcLength = theta * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;

        return arcLength;
    }

    public int getTargetClicks(){
        return targetClicks;
    }

    public double distanceRemaining(int currClick){
        double delta = (targetClicks - currClick) * constants.INCHES_PER_CLICK;

        return delta;
    }


    public double getAngleRemaining(double currR){
        double target2 = (theta < 0 ? theta + 360 : theta);
        double current2 = (currR < 0 ? currR + 360 : currR);

        double turnAmount1 = theta - currR;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        return turnAmount;
    }
}
