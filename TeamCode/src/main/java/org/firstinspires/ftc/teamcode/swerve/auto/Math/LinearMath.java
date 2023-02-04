package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;

public class LinearMath { //Note: snap() is used in the auto class separately. This class is used assuming that the wheels are already pointing the way we want it to.
    Constants constants = new Constants();

    private int targetClicks;
    private double targetDistance = 0;

    private double x;
    private double y;
    private double theta; //amount robot header should turn (for table-spinning)

    public LinearMath(){
    }

    public void setPos(double x, double y, double target, double current){
        this.x = x;
        this.y = y;
        this.theta = SwervePod.changeAngle(target, current);

        targetClicks = (int)(getDistance() * constants.CLICKS_PER_INCH);
        targetDistance = getDistance();
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double getTargetDistance(){
        return targetDistance;
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

    public double distanceRemaining(double distanceRan){
        return targetDistance - distanceRan;
    }

    public double getRunTime(double rate){
        return (getDistance() * constants.CLICKS_PER_INCH) / rate;
    }
}
