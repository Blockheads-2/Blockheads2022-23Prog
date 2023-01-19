package org.firstinspires.ftc.teamcode.common.pid;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.SwervePod;

public class HeaderControlPID {
    //the purpose of this class is to keep the header pointint straight.

    Constants constants = new Constants();

    SwervePod.Side throttleSide;

    private double throttle;
    public double error = 0;
    public double prevError = 0;
    public double biggerArc = 0;

    private int[] prevClicksPos = new int[4];
    private double prevTranslateR = 0;
    private double prevTranslateL = 0;

    public double target = 0;

    public HeaderControlPID(int[] clicksPos){
        prevClicksPos = clicksPos;
    }

    public void calculateThrottle(int[] clicksPos, double currentR, double targetTheta, boolean reset){
        //right
        int topR = clicksPos[2] - prevClicksPos[2]; //change in top right
        int botR = clicksPos[3] - prevClicksPos[3]; //change in bottom right
        double translateClicksR = (topR - botR) / 2.0;

        //left
        int topL = clicksPos[0] - prevClicksPos[0]; //change in top left
        int botL = clicksPos[1] - prevClicksPos[1]; //change in bottom left
        double translateClicksL = (topL - botL) / 2.0;

        double deltaLeft = translateClicksL - prevTranslateL;
        double deltaRight = translateClicksR - prevTranslateR;  //does it matter if it's negative or not?
        biggerArc = (Math.abs(deltaLeft) > Math.abs(deltaRight) ? deltaLeft : deltaRight);

        if (reset){
            prevClicksPos = clicksPos;
            prevTranslateL = translateClicksL;
            prevTranslateR = translateClicksR;
            throttle = 1.0;
            return;
        }else if (deltaLeft == deltaRight) {
            throttle = 1.0;
            return;
        } else if (biggerArc == deltaLeft) throttleSide = SwervePod.Side.LEFT;
        else throttleSide = SwervePod.Side.RIGHT;

        prevError = error;
        error = SwervePod.changeAngle(targetTheta, currentR);

        throttle = 1.0 - ((2.0 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER * Math.abs(error)) / Math.abs(biggerArc));

        if ((prevError < 0 ? -1 : 1) != (error < 0 ? -1 : 1)){ //if the error changes signs, then that means that the greater arc has changed.
            prevClicksPos = clicksPos;
            prevTranslateL = translateClicksL;
            prevTranslateR = translateClicksR;
        }
    }

    public void calculateThrottleDemoFuncTestLater(int[] clicksPos, double currentR, double targetTheta, boolean reset){
        //right
        int topR = clicksPos[2] - prevClicksPos[2]; //change in top right
        int botR = clicksPos[3] - prevClicksPos[3]; //change in bottom right
        double translateClicksR = (topR - botR) / 2.0;

        //left
        int topL = clicksPos[0] - prevClicksPos[0]; //change in top left
        int botL = clicksPos[1] - prevClicksPos[1]; //change in bottom left
        double translateClicksL = (topL - botL) / 2.0;

        double deltaLeft = translateClicksL - prevTranslateL;
        double deltaRight = translateClicksR - prevTranslateR;  //does it matter if it's negative or not?
        biggerArc = (Math.abs(deltaLeft) > Math.abs(deltaRight) ? deltaLeft : deltaRight);

        if (reset){
            prevClicksPos = clicksPos;
            prevTranslateL = translateClicksL;
            prevTranslateR = translateClicksR;
            throttle = 1.0;
            return;
        }else if (deltaLeft == deltaRight) {
            throttle = 1.0;
            return;
        } else if (biggerArc == deltaLeft) throttleSide = SwervePod.Side.LEFT;
        else throttleSide = SwervePod.Side.RIGHT;

        error = Math.abs(SwervePod.changeAngle(targetTheta, currentR));

        throttle = 1.0 - ((2.0 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER * error) / biggerArc);

        if (Math.abs(error) < constants.degreeTOLERANCE){
            prevClicksPos = clicksPos;
            prevTranslateL = translateClicksL;
            prevTranslateR = translateClicksR;
        }
    }

    public double getThrottle(SwervePod.Side side){
        throttle = Math.abs(throttle);
        if (throttle > 1) throttle = 1;

        if (this.throttleSide == side) return Math.abs(throttle);
        else return 1.0;
    }

    public SwervePod.Side getThrottleSide(){
        return throttleSide;
    }

}
