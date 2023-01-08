package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;

import java.util.HashMap;

public class SwervePod {
    Constants constants = new Constants();

    public enum Side{
        RIGHT,
        LEFT
    }
    Side side;

    SnapSwerveModulePID pid;

    private double currentW = 0;

    private double throttle = 0;

    private double power = 0;

    private double turnAmount = 0;

    HashMap<String, Double> output = new HashMap<String, Double>();

    public SwervePod(Side side){
        this.side = side;
        pid = new SnapSwerveModulePID();
    }

    public void setPower(double p){
        power = p;
    }

    public void setCurrents(double currentW){
        this.currentW = currentW;
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        throttle = constants.initDirectionRight;

//        posSystem.setOptimizedCurrentW(leftCurrentW, rightCurrentW);

        if(Math.abs(turnAmount) > 90){
            double temp_target = clamp(target + 180);
            turnAmount = temp_target - currentW;

            throttle *= -1;
        }
        return turnAmount;
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;
        if (degrees == -180) degrees = 180;

        if (degrees < -180){
            degrees = 180 - (Math.abs(degrees) - 180);
        } else if (degrees > 180){
            degrees = -180 + (Math.abs(degrees) - 180);
        }
        return degrees;
    }
}
