package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class TurnMath {
    Constants constants = new Constants();

    private double theta;
    private double arcLength;

    private int targetLClicks;
    private int targetRClicks;

    private int direction;

    public void setPos(double theta, int initClicksL, int initClicksR, int turnDirection){
        this.theta = theta;

        targetLClicks = (int)(getDistance() * constants.CLICKS_PER_INCH * turnDirection) + initClicksL;
        targetRClicks = -targetLClicks * turnDirection + initClicksR;

        direction = (targetLClicks > targetRClicks ? -1 : 1);

        this.arcLength = Math.abs(getDistance());
    }

    public double getDistance(){
        double arcLength = theta * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;

        return arcLength;
    }

    public int getTargetLClicks(){
        return targetLClicks;
    }

    public int getTargetRClicks(){
        return targetRClicks;
    }

    public double getDistanceRemaining(int currClickL, int currClickR){
        int clicksRemainingL = (targetLClicks - currClickL);
        int clicksRemainingR = (targetRClicks - currClickR);
        int clicksRemaining = (direction == 1 ? clicksRemainingR : clicksRemainingL);

        return clicksRemaining;
    }

    public double distanceRemainingR(int currClickR){
        double deltaR = (targetRClicks - currClickR) * constants.INCHES_PER_CLICK;

        return deltaR;
    }

    public double distanceRemainingL(int currClickL){
        double deltaL = (targetLClicks - currClickL) * constants.INCHES_PER_CLICK;

        return deltaL;
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
