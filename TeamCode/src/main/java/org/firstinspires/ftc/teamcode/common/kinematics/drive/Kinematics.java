package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

public class Kinematics {
    protected Constants constants = new Constants();

    public enum DriveType{
        LINEAR,
        SNAP,
        SPLINE,
        TURN, //robot turns on its center
        STOP,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    protected double leftRotatePower = 0.0;
    protected double rightRotatePower = 0.0;
    protected double spinPower = 0.0;
    protected double translationPowerPercentage = 0.0;
    protected double rotationPowerPercentage = 0.0;
    protected double leftThrottle = 1.0;
    protected double rightThrottle = 1.0;
    protected int translateSwitchMotors = 1; //1 if going forward, -1 if going backward
    protected int leftTurnDirectionW = -1;
    protected int rightTurnDirectionW = 1;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicks = 0; //make protected later

    public int splineSpinClicksL = 0; //can combine with spinClicks later (spinClicksR and spinClicksL)
    public int splineSpinClicksR = 0;

    //current orientation
    protected GlobalPosSystem posSystem;
    protected double leftCurrentW; //current wheel orientation
    protected double rightCurrentW;
    protected double currentR; //current robot header orientation

    //PIDs
    protected RotateSwerveModulePID snapLeftWheelPID;
    protected RotateSwerveModulePID snapRightWheelPID;
    protected RotateSwerveModulePID tableSpinWheelPID;
//    protected LinearCorrectionPID linearCorrectionPID;
//    protected RotateSwerveModulePID counteractSplinePID;

    //targets
    protected double leftTargetW = 0.0;
    protected double rightTargetW = 0.0;
    protected double leftOptimizedTargetW = 0.0;
    protected double rightOptimizedTargetW = 0.0;
    protected double rightTurnAmountW = 0.0;
    protected double leftTurnAmountW = 0.0;
    protected double targetR = 0.0;
//    protected double splineReference = 0.0;

    public Kinematics(GlobalPosSystem posSystem){
        snapLeftWheelPID = new RotateSwerveModulePID();
        snapRightWheelPID = new RotateSwerveModulePID();
//        linearCorrectionPID = new LinearCorrectionPID();
        tableSpinWheelPID = new RotateSwerveModulePID();
//        counteractSplinePID = new RotateSwerveModulePID();
        this.posSystem = posSystem;
    }

    public void setCurrents(){
//        leftCurrentW = posSystem.getLeftWheelW();
        rightCurrentW = posSystem.getRightWheelW();
        currentR = posSystem.getPositionArr()[4];
    }

//    public void setSplineReference(){
//        splineReference = posSystem.getPositionArr()[3];
//        counteractSplinePID.setTargets(splineReference, 0.05, 0, 0.02);
//    }

    public double[] wheelOptimization(double x, double y, double currentW){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target = Math.toDegrees(Math.atan2(x, y));
        if (x==0 && y==0) target = 0;
        directionArr[1] = target;

        //determine how much modules must turn in which direction (optimization)
        double turnAmount = target - currentW;
        double turnDirection = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }

        if(Math.abs(turnAmount) > 90){
            target += 180;
            target = clamp(target);
            turnAmount = target - currentW;
            turnDirection *= -1;
            if(Math.abs(turnAmount) > 180){
                turnAmount = 360 - Math.abs(turnAmount);
            }
            this.translateSwitchMotors *= -1;
        }
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = turnDirection;

        return directionArr;
    }

    public double[] robotHeaderOptimization(double x, double y){
        double[] directionArr = new double[2];

        //determine targets
        double target = Math.toDegrees(Math.atan2(x, y));
        directionArr[1] = target;

        //determine how much robot header must turn in which direction
        double turnAmount = target-currentR;
        double turnDirection = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        directionArr[0] = (Math.abs(turnAmount) * turnDirection);


        return directionArr;
    }

    public double robotHeaderOptimization(double targetOrientation){
        //determine how much robot header must turn in which direction
        double turnAmount = targetOrientation-currentR;
        double turnDirection = Math.signum(turnAmount);
        if(Math.abs(turnAmount) > 180) {
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }
        return (turnAmount * turnDirection);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }


    public DriveType getDriveType(){
        return type;
    }
}

