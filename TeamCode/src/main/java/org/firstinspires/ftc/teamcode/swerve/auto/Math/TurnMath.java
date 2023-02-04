package org.firstinspires.ftc.teamcode.swerve.auto.Math;

import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;

public class TurnMath {
    Constants constants = new Constants();

    private double theta;

    private double targetDistance;

    public void setPos(double target, double current, int currentDirection, boolean rightPod){
        this.theta = SwervePod.changeAngle(target, current);
        targetDistance = getDistance() * currentDirection * (rightPod ? -1 : 1);
    }

    public double getDistance(){
        double arcLength = Math.toRadians(theta) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;

        return arcLength;
    }

    public double distanceRemaining(double distanceRan){
        return targetDistance - distanceRan;
    }

    public double getAngleRemaining(double currR){
        return SwervePod.changeAngle(this.theta, currR);
    }
}
