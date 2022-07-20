package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

public class Kinematics {
    Constants constants = new Constants();

    public enum DriveType{
        LINEAR,
        SNAP,
        SPLINE,
        TURN, //robot header turns
        STOP
    }
    private DriveType type;

    //robot's power
    private double rotatePower = 0.0;
    public double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 1.0;
    private double rightThrottle = 1.0;
    private int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    private int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //current orientation
    private double currentW; //current wheel orientation
    private double currentR; //current robot header orientation

    //PIDs
    private RotateSwerveModulePID snapWheelPID;
    private RotateSwerveModulePID tableSpinWheelPID;

    //targets
    double x = 0.0;
    double y = 0.0;
    private double wheelTurnAmount = 0.0; //how much the wheels should rotate
    private double robotTurnAmount = 0.0; //how much the robot should turn
    double targetW = 0.0;
    double targetR = 0.0;

    //checkover
    private boolean finished_stop = false;
    private boolean finished_snap = false;
    public boolean dontspline = false;

    public Kinematics(){

    }

    public void logic(){
        rotatePower = snapWheelPID.update(currentW);

        switch(type){
            case LINEAR:
                if (!finished_stop) type = DriveType.STOP;

                if (finished_stop && !finished_snap){
                    type = DriveType.SNAP;
                } else if (finished_stop && finished_snap){
                    rightThrottle = 1;
                    leftThrottle = 1;
                    translateSwitchMotors = rotationSwitchMotors;
                    translationPowerPercentage = 1.0;
                    rotationPowerPercentage = 0.0;
                }
                break;

            case SNAP:
                if (Math.abs(currentW - targetW) <= constants.TOLERANCE){ //rotate modules until target is hit
                    rightThrottle = 1;
                    leftThrottle = 1;
                    rotationSwitchMotors = (int)getWheelDirection(x, y)[2];
                    translateSwitchMotors = rotationSwitchMotors;
                    translationPowerPercentage = 0.0;
                    rotationPowerPercentage = 1.0;
                    rotatePower = snapWheelPID.update(currentW);
                } else {
                    finished_snap = true;
                    dontspline = false;
                }
                break;

            case SPLINE:
                translationPowerPercentage = 1.0;
                rotationPowerPercentage = 0.0;
                double throttle = Math.tanh(Math.abs(y / (2 * x)));

                if (Math.abs(currentR - targetR) <= constants.TOLERANCE){
                    rightThrottle = (rotationSwitchMotors == 1 ? throttle : 1);
                    leftThrottle = (rotationSwitchMotors == 1 ? 1 : throttle);
                } else{
                    rightThrottle = 1; //once target is met, stop splining and just move straight
                    leftThrottle = 1;
                }

                finished_stop = false;
                finished_snap = false;

                break;

            case TURN:
                //...
                finished_stop = false;
                finished_snap = false;

                break;

            case STOP:
                rightThrottle = 0;
                leftThrottle = 0;
                finished_stop = true;
                dontspline = true;
                break;
        }
    }

    private void tableSpin(){ //NNEEEDSSS REVISSIONNN
        //rotation and translation power percentages
        rotatePower = tableSpinWheelPID.update(currentR);
        rotationSwitchMotors = (int)getRobotDirection(right_stick_x, right_stick_y)[1];
        if (Math.abs(targetR - currentR) <= constants.TOLERANCE){ //while target not hit
            rotationPowerPercentage = rotatePower / 1.4; //1.4 can be changed based on testing
            translationPowerPercentage = 1 - rotationPowerPercentage;
        } else{ //after target is hit, stop table spinning
            rotationPowerPercentage = 0.0;
            translationPowerPercentage = 1.0;
        }
    }

    public void setPos(DriveType type, double x, double y, double wheelTurnAmount, double robotTurnAmount){
        this.type = type;
        this.x = x;
        this.y = y;
        this.wheelTurnAmount = wheelTurnAmount;
        this.robotTurnAmount = robotTurnAmount;
        targetW = currentW + wheelTurnAmount;
        targetR = currentR + robotTurnAmount;
        if (targetW < -179 || targetW > 180) targetW -= (Math.signum(targetW) * 360);
        if (targetR < -179 || targetR > 180) targetR -= (Math.signum(targetR) * 360);
        snapWheelPID = new RotateSwerveModulePID(targetW, 0, 0, 0); //does this make a new object every loop?
        tableSpinWheelPID = new RotateSwerveModulePID(targetR, 0, 0, 0);
    }

    public void setCurrents(double currentWheelOrientation, double currentRobotOrientation){
        currentW = currentWheelOrientation;
        currentR = currentRobotOrientation;
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

    public int getClicks(){
        double translationClicks = getDistance() * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = getRobotDirection(x, y)[0] * constants.CLICKS_PER_DEGREE; //table spinning clicks

        return (int)(translationClicks + rotationClicks);
    }

    public double[] getWheelDirection(double x, double y){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target =  Math.toDegrees(Math.atan2(x, y)); //finds target orientation in terms of degrees (range is (-180, 180])
        directionArr[1] = target;

        //determine how much modules must turn
        if (currentW > target) target += 360;
        double turnAmount = Math.abs(target - currentW);
        if (turnAmount > 90) turnAmount = 90 - (turnAmount%90);
        directionArr[0] = turnAmount;

        //determine direction wheel will rotate
        double switchMotors = 1; //"by default, everything will rotate right."
        switchMotors *= (turnAmount <= 90 ? 1 : -1); //1 = right, -1 = left
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public double[] getRobotDirection(double x, double y){
        double[] directionArr = new double[3];

        //determine targets
        double target = Math.toDegrees(Math.atan2(x, y));
        directionArr[1] = target;

        //determine how much robot header must turn in which direction
        double turnAmount = target-currentR;
        double switchMotors = Math.signum(turnAmount);
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public double[] getVelocity(){
        double[] motorPower = new double[4];

        motorPower[0] = constants.MAX_VELOCITY_DT * leftThrottle * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top left
        motorPower[1] = constants.MAX_VELOCITY_DT * leftThrottle * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom left
        motorPower[2] = constants.MAX_VELOCITY_DT * rightThrottle * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top right
        motorPower[3] = constants.MAX_VELOCITY_DT * rightThrottle * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom right

        return motorPower;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];

        motorPower[0] = (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * leftThrottle; //top left
        motorPower[1] = (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * leftThrottle; //bottom left
        motorPower[2] = (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * rightThrottle; //top right
        motorPower[3] = (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * rightThrottle ; //bottom right

        return motorPower;
    }

    public boolean resetStuff(){
        if (currentW != 0){
            rotationSwitchMotors = (currentW > 0 ? -1 : 1); //1 = right, -1 = left

            RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(0, 0, 0, 0);
            rotatePower = rotateWheelPID.update(currentW);

            translationPowerPercentage = 0.0;
            rotationPowerPercentage = 1.0;
            leftThrottle = 1.0;
            rightThrottle = 1.0;
            spinPower = 0.0;
            translateSwitchMotors = 1;
        } else{
            translationPowerPercentage = 0.0;
            rotationPowerPercentage = 0.0;
            rotatePower = 0.0;
            rotationSwitchMotors = 1;
        }
        return (currentW == 0);
    }
}

