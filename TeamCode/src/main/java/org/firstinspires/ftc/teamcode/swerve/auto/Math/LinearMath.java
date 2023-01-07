package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class LinearMath { //Note: snap() is used in the auto class separately. This class is used assuming that the wheels are already pointing the way we want it to.
    Constants constants = new Constants();

    private double initialX;
    private double initialY;

    private double x;
    private double y;
    private double theta; //amount robot header should turn (for table-spinning)


    public LinearMath(){
    }

    public void setInits(double x, double y){
        initialX = x;
        initialY = y;
    }

    public void setPos(double x, double y, double theta, double kp, double ki, double kd){
        this.x = x;
        this.y = y;
        this.theta = theta;
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

    public double distanceRemaining(double currentX, double currentY){
        return (Math.sqrt(Math.pow(this.x - currentX, 2) + Math.pow(this.y - currentY, 2)));
    }

    public double getRunTime(double rate){
        return (getDistance() * constants.CLICKS_PER_INCH) / rate;
    }
}
