package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class TurnMath {
    Constants constants = new Constants();

    private double theta;
    private double arcLength;

    SpinPID spinPID = new SpinPID();

    public void setPos(double theta, double kp, double ki, double kd){
        this.theta = theta;
        this.arcLength = getDistance();

        spinPID.setTargets(this.arcLength, kp, ki, kd);
    }

    public double getDistance(){
        return theta * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;
    }

    public double getDistanceRemaining(double currR){
        double target2 = (theta < 0 ? theta + 360 : theta);
        double current2 = (currR < 0 ? currR + 360 : currR);

        double turnAmount1 = theta - currR;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        return turnAmount;
    }

}
