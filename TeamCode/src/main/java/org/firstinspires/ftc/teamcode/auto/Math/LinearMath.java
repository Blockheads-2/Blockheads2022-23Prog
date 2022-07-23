package org.firstinspires.ftc.teamcode.auto.Math;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Kinematics;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class LinearMath { //Note: snap() is used in the auto class separately. This class is used assuming that the wheels are already pointing the way we want it to.
    Constants constants = new Constants();
    SpinPID spinPID = new SpinPID();

    private double initialX;
    private double initialY;

    private double x;
    private double y;
    private double theta; //amount robot header should turn (for table-spinning)



    public LinearMath(double initX, double initY){
        initialX = initX;
        initialY = initY;
    }

    public void setPos(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
        spinPID.setTargets(getDistance(), 0, 0, 0);
    }

    private double getDistance(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public int getClicks(){ //needs work
        double translationClicks = getDistance() * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = theta * constants.CLICKS_PER_DEGREE; //table spinning clicks

        return (int)(translationClicks + rotationClicks);
    }

    public double getSpinPower(double currentX, double currentY){
        double distanceTravelled = Math.sqrt(Math.pow(currentX - initialX, 2) + Math.pow(currentY - initialY, 2));

        return spinPID.update(distanceTravelled);
    }
}
