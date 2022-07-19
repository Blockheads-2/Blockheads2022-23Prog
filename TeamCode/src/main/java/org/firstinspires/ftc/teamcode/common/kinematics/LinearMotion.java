package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

public class LinearMotion {
    Constants constants = new Constants();

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    private int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //wheel target
    private double wheelOrientation = 0.0;
    private double wheelTargetAmountTurned;

    //current orientation
    private double currentW;
    private double currentR;

    //robot header target
    private double robotOrientation = 0.0;

    //PIDs
    RotateSwerveModulePID snapWheelPID;
    RotateSwerveModulePID tableSpinWheelPID;

    //targets
    double x = 0.0;
    double y = 0.0;
    double finalAngle = 0.0; //how much the robot should turn, relative to its starting position

    //checkover
    private int tolerance = 3; //number of clicks the motors can tolerate to be off by (this number will be determined during testing)
    private boolean stop_snap = false; //checks if the robot is stop_wheel_snapping its wheels to a desired orientation

    public LinearMotion(){

    }

    public void logic(){
        //rotation and translation power percentages
//        double tableSpinPower = Math.sqrt(Math.pow(right_stick_x, 2) + Math.pow(right_stick_y, 2));
//        rotationSwitchMotors = (int)robotDirection(right_stick_x, right_stick_y, robotOrientation)[1];
//        if (Math.abs(currentR - robotOrientation) <= constants.TOLERANCE){ //while target not hit
//            rotationPowerPercentage = tableSpinPower / 1.4; //1.4 can be changed based on testing
//            translationPowerPercentage = 1 - rotationPowerPercentage;
//        } else{ //after target is hit, stop table spinning
//            rotationPowerPercentage = 0.0;
//            translationPowerPercentage = 1.0;
//        }
    }

    public void setPos(double x, double y, double finalAngle, double currentW, double currentR){
        this.x = x;
        this.y = y;
        this.finalAngle = finalAngle;
        this.currentW = currentW;
        this.currentR = currentR;
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

    public int getClicks(){
        double translationClicks = getDistance() * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = robotDirection(x, y, currentR)[0] * constants.CLICKS_PER_DEGREE; //table spinning clicks

        return (int)(translationClicks + rotationClicks);
    }

    private double[] wheelDirection(double x, double y, double current){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target =  Math.toDegrees(Math.atan2(x, y)); //finds target orientation in terms of degrees (range is (-180, 180])
        directionArr[1] = target;

        //determine how much modules must turn
        if (current > target) target += 360;
        double turnAmount = Math.abs(target - current);
        if (turnAmount > 90) turnAmount = 90 - (turnAmount%90);
        directionArr[0] = turnAmount;

        //determine direction wheel will rotate
        double switchMotors = 1; //"by default, everything will rotate right."
        switchMotors *= (turnAmount <= 90 ? 1 : -1); //1 = right, -1 = left
        directionArr[2] = switchMotors;

        return directionArr;
    }

    private double[] robotDirection(double x, double y, double current){
        double[] directionArr = new double[3];

        //determine targets
        double target = Math.toDegrees(Math.atan2(x, y));
        directionArr[1] = target;

        //determine how much robot header must turn in which direction
        double turnAmount = target-current;
        double switchMotors = Math.signum(turnAmount);
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public double[] getVelocity(){
        double[] motorPower = new double[4];

        motorPower[0] = constants.MAX_VELOCITY_DT * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top left
        motorPower[1] = constants.MAX_VELOCITY_DT * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom left
        motorPower[2] = constants.MAX_VELOCITY_DT * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top right
        motorPower[3] = constants.MAX_VELOCITY_DT * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom right

        return motorPower;
    }
}

