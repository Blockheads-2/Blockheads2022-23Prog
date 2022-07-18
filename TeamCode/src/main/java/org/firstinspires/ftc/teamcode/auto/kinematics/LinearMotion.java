package org.firstinspires.ftc.teamcode.auto.kinematics;

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

    }

    public void setPos(double x, double y, double finalAngle){
        this.x = x;
        this.y = y;
        this.finalAngle = finalAngle;
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }


    public double[] targets(double currentOrientation){ //returns how much the robot should turn in which direction
        double[] directionArr = new double[3];
        double switchMotors = 1; //"by default, everything will rotate right."

        double targetOrientation = Math.toDegrees(Math.atan2(x, y)); //finds target orientation in terms of degrees (range is (-pi, pi])
        double temp_JtargetOrientation = targetOrientation;
        if (currentOrientation > targetOrientation) targetOrientation += 360;
        double targetAmountTurned = Math.abs(targetOrientation - currentOrientation); //how much the robot/wheel must turn
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //1 = right, -1 = left
        targetOrientation = temp_JtargetOrientation;

        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);

        directionArr[0] = targetAmountTurned;
        directionArr[1] = targetOrientation;
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

