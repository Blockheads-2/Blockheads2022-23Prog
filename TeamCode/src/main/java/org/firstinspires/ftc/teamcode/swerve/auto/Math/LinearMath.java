package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class LinearMath { //Note: snap() is used in the auto class separately. This class is used assuming that the wheels are already pointing the way we want it to.
    Constants constants = new Constants();

    private int targetClicksR;
    private int targetClicksL;

    private double x;
    private double y;
    private double theta; //amount robot header should turn (for table-spinning)


    public LinearMath(){
    }

    public void setPos(double x, double y, double theta, double kp, double ki, double kd, int initClickL, int initClickR){
        this.x = x;
        this.y = y;
        this.theta = theta;

        targetClicksR = (int)(getDistance() * constants.CLICKS_PER_INCH) + initClickR;
        targetClicksL = (int)(getDistance() * constants.CLICKS_PER_INCH) + initClickL;
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        double translationClicks = getDistance() * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = theta * constants.CLICKS_PER_DEGREE; //table spinning clicks

        clicks[0] = (int)(translationClicks + rotationClicks);
        clicks[1] = (int)(-translationClicks + rotationClicks);
        clicks[2] = (int)(translationClicks + rotationClicks);
        clicks[3] = (int)(-translationClicks + rotationClicks);
        return clicks;
    }

    public double distanceRemainingR(int currClickR){
        double deltaR = (targetClicksR - currClickR) * constants.INCHES_PER_CLICK;

        return deltaR;
    }

    public double distanceRemainingL(int currClickL){
        double deltaL = (targetClicksL - currClickL) * constants.INCHES_PER_CLICK;

        return deltaL;
    }


    public double getRunTime(double rate){
        return (getDistance() * constants.CLICKS_PER_INCH) / rate;
    }
}
