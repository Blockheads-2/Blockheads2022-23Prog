package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class TurnMath {
    Constants constants = new Constants();

    private double theta;
    private double arcLength;
    private double initClicksR;

    SpinPID spinPID = new SpinPID();

    public void setPos(double theta, double kp, double ki, double kd, double initClicksR){
        this.theta = theta;
        this.arcLength = getDistance();

        spinPID.setTargets(this.arcLength, kp, ki, kd);
        this.initClicksR = initClicksR;
    }

    public double getDistance(){
        return theta * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;
    }

    public double getDistanceLeft(double currClickR){
        double delta = (currClickR - initClicksR) * constants.INCHES_PER_CLICK;
        return (arcLength - delta);
    }

    public double getPower(double current){
        return (spinPID.update(current-initClicksR));
    }
}
