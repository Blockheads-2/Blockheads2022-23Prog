package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class SplineMath {
    Constants constants = new Constants();
    SpinPID spinPIDR;
    SpinPID spinPIDL;

    private double x;
    private double y;
    private double radius;
    private double theta;
    private double turnAmount; //amount robot header should turn (for table-spinning)
    private double distanceR;
    private double distanceL;

    private int targetClicksR;
    private int targetClicksL;

    public SplineMath(){
        spinPIDR = new SpinPID();
        spinPIDL = new SpinPID();
    }

    public void setPos(double x, double y, double theta, double kp, double ki, double kd, int initClickL, int initClickR){
        this.x = x;
        this.y = y;
        turnAmount = theta;

        calculateDistance();

        targetClicksL = initClickL + (int)(distanceL * constants.CLICKS_PER_INCH);
        targetClicksR = initClickR + (int)(distanceR * constants.CLICKS_PER_INCH);

        spinPIDL.setTargets(distanceL, kp, ki, kd);
        spinPIDR.setTargets(distanceR, kp, ki, kd);
    }

    public void calculateDistance(){
        radius = ((x * x) + (y * y)) / (2 * x);
        theta = 0;

        if (x == 0){ //linear movement if x=0. No splining
            distanceL = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            distanceR = distanceL;
            return;
        }

        try{
            radius = ((x * x) + (y * y)) / (2 * x);
            theta = Math.acos(
                    1 - (
                            ((x * x) + (y * y)) / (2 * radius * radius)
                    )
            );
        } catch (ArithmeticException e){
//            System.out.println("Error: " + e);
        }

        distanceL = (radius + constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) * theta; //left distance
        distanceR = (radius - constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) * theta; //right distance
    }

    public double distanceRemainingR(int currClickR){
        double deltaR = (targetClicksR - currClickR) * constants.INCHES_PER_CLICK;

        return deltaR;
    }

    public double distanceRemainingL(int currClickL){
        double deltaL = (targetClicksL - currClickL) * constants.INCHES_PER_CLICK;

        return deltaL;
    }

    public double[] getDistance(){
        return new double[]{distanceL, distanceR};
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        int spinClicksR = (int)(distanceR * constants.CLICKS_PER_INCH);
        int spinClicksL = (int)(distanceL * constants.CLICKS_PER_INCH);
        int rotationClicks = (int)(turnAmount * constants.CLICKS_PER_DEGREE); //table spinning clicks

        clicks[0] = spinClicksL + rotationClicks; //left side
        clicks[1] = -spinClicksL + rotationClicks; //left side

        clicks[2] = spinClicksR + rotationClicks; //right side
        clicks[3] = -spinClicksR + rotationClicks; //right side
        return clicks;
    }

//    public double returnPowerR(int currentClickR){
//        double distanceGap = Math.abs(distanceRemainingR(currentClickR));
//        return (distanceR >= distanceL ? 1 * spinPIDR.update(distanceGap): (distanceL / distanceR) * spinPIDR.update(distanceGap));
//    }
//
//    public double returnPowerL(int currentClickL){
//        double distanceGap = Math.abs(distanceRemainingL(currentClickL));
//        return (distanceL >= distanceR ? 1 * spinPIDL.update(distanceGap) : (distanceR / distanceL) * spinPIDL.update(distanceGap));
//    }

    public double getRunTime(double rate){
        return (radius * theta * constants.CLICKS_PER_INCH) / rate;
    }
}