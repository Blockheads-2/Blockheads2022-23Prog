package org.firstinspires.ftc.teamcode.swerve.common.pid;

import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;

public class HeaderControlPID {
    //the purpose of this class is to keep the header pointint straight.

    Constants constants = new Constants();

    SwervePod.Side throttleSide;

    private double throttle;
    public double error = 0;
    public double prevError = 0;
    public double biggerArc = 0;

    private double prevTranslateR = 0;
    private double prevTranslateL = 0;

    public double target = 0;

    public HeaderControlPID(){
    }

    public void reset(double distanceL, double distanceR){
        prevTranslateL = distanceL;
        prevTranslateR = distanceR;
        throttle = 1.0;
        error = 0;
        prevError = 0;
    }

    public void calculateThrottle(double distanceL, double distanceR, double currentR, double targetTheta){
        double deltaLeft = distanceL - prevTranslateL;
        double deltaRight = distanceR - prevTranslateR;  //does it matter if it's negative or not?
        biggerArc = (Math.abs(deltaLeft) > Math.abs(deltaRight) ? deltaLeft : deltaRight);

        if (deltaLeft == deltaRight) {
            throttle = 1.0;
            return;
        } else if (biggerArc == deltaLeft) throttleSide = SwervePod.Side.LEFT;
        else throttleSide = SwervePod.Side.RIGHT;

        prevError = error;
        error = SwervePod.changeAngle(targetTheta, currentR);

        throttle = 1.0 - ((2.0 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER * Math.abs(error)) / Math.abs(biggerArc));

        if (throttle > 1) throttle = 1;
        else if (throttle < 0) throttle = 0;

        if ((prevError < 0 ? -1 : 1) != (error < 0 ? -1 : 1)){ //if the error changes signs, then that means that the bigger arc has changed sides.
            prevTranslateL = distanceL;
            prevTranslateR = distanceL;
        }
    }

    public double getThrottle(SwervePod.Side side, boolean specialSpliningCondition){
        if (!specialSpliningCondition){
            throttle = Math.abs(throttle);
            if (throttle > 1) throttle = 1;

            if (this.throttleSide == side) return Math.abs(throttle);
            else return 1.0;
        } else return 1.0;
    }

    public SwervePod.Side getThrottleSide(){
        return throttleSide;
    }

}
