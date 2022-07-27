package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class Kinematics {
    Constants constants = new Constants();
    SplineMath splinemath;
    LinearMath linearmath;

    public enum DriveType{
        LINEAR,
        SNAP,
        SPLINE,
        TURN, //robot turns on its center
        STOP,
        NOT_INITIALIZED
    }
    private DriveType type = DriveType.NOT_INITIALIZED;

    public enum Mode{AUTO, TELEOP};
    private Mode mode = Mode.AUTO;

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 1.0;
    private double rightThrottle = 1.0;
    private double speed = 0.0; //for autoMode (between 0~1)
    private int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    private int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //current orientation
    GlobalPosSystem posSystem;
    private double currentW; //current wheel orientation
    private double currentR; //current robot header orientation

    //PIDs
    private RotateSwerveModulePID snapWheelPID;
    private RotateSwerveModulePID tableSpinWheelPID;
    private RotateSwerveModulePID resetWheelPID;

    //targets
    double x = 0.0;
    double y = 0.0;
    private double wheelTurnAmount = 0.0; //how much the wheels should rotate
    private double robotTurnAmount = 0.0; //how much the robot should turn
    double targetW = 0.0;
    double targetR = 0.0;

    //checkover
    private boolean finished_stop = false;
    public boolean finished_snap = false;
    public boolean dontspline = false;
    public boolean firstMovement = true;


    public Kinematics(GlobalPosSystem posSystem){
        snapWheelPID = new RotateSwerveModulePID();
        tableSpinWheelPID = new RotateSwerveModulePID();
        resetWheelPID = new RotateSwerveModulePID();
        this.posSystem = posSystem;
    }

    public void logic(){
        rotatePower = snapWheelPID.update(currentW);

        switch(type){
            case LINEAR:
                if (!finished_stop && mode == Mode.TELEOP) type = DriveType.STOP;

                if (finished_stop && !finished_snap && mode == Mode.TELEOP){
                    type = DriveType.SNAP;
                }
                if ((finished_stop && finished_snap) || mode == Mode.AUTO){
                    finished_stop = true;
                    finished_snap = true;
                    rightThrottle = 1;
                    leftThrottle = 1;
                    translateSwitchMotors = rotationSwitchMotors;
                    translationPowerPercentage = 1.0;
                    rotationPowerPercentage = 0.0;
                    setSpinPower();
                    tableSpin();
                    firstMovement = false;
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
                setSpinPower();
                finished_stop = false;
                finished_snap = false;
                tableSpin();
                break;
                /*
                To Do:
                Spline w/ a constant heading
                 */

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
                firstMovement = true;
                break;
        }
    }

    private void tableSpin(){
        //rotation and translation power percentages
        rotatePower = tableSpinWheelPID.update(currentR);
        rotationSwitchMotors = (int)getRobotDirection(targetR)[2];
        if (Math.abs(targetR - currentR) <= constants.TOLERANCE){ //while target not hit
            rotationPowerPercentage = rotatePower / 1.4; //1.4 can be changed based on testing
            translationPowerPercentage = 1 - rotationPowerPercentage;
        } else{ //after target is hit, stop table spinning
            rotationPowerPercentage = 0.0;
            translationPowerPercentage = 1.0;
        }
    }


    public void setPos(DriveType type, double x, double y, double robotTurnAmount, double speed){
        this.type = type;
        this.x = x;
        this.y = y;
        this.speed = speed;

        //setting targets
        this.wheelTurnAmount = getWheelDirection(x, y)[0];
        this.robotTurnAmount = robotTurnAmount; //how much the robot should turn
        targetW = currentW + wheelTurnAmount;
        targetR = currentR + robotTurnAmount; //the target orientation on a circle of (-179, 180]
        if (targetW < -179 || targetW > 180) targetW -= (Math.signum(targetW) * 360);
        if (targetR < -179 || targetR > 180) targetR -= (Math.signum(targetR) * 360);

        //setting PIDs for rotation of wheels & robot
        snapWheelPID.setTargets(targetW, 0, 0, 0);
        tableSpinWheelPID.setTargets(targetR, 0, 0, 0);

        //specifically for autoMode (not teleop)
        splinemath = new SplineMath(posSystem.getMotorClicks()[2], posSystem.getMotorClicks()[0]);
        splinemath.setPos(x, y, robotTurnAmount);
        linearmath = new LinearMath(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
        linearmath.setPos(x, y, robotTurnAmount);
    }

    public void setSpinPower(){
        if (mode == Mode.TELEOP) {
            spinPower = Math.sqrt(Math.pow(x,2) + Math.pow(y, 2));

            if (Math.abs(currentR - targetW) <= constants.TOLERANCE && type == DriveType.SPLINE){
                double throttle = Math.tanh(Math.abs(y / (2 * x)));
                rightThrottle = (rotationSwitchMotors == 1 ? throttle : 1);
                leftThrottle = (rotationSwitchMotors == 1 ? 1 : throttle);
            } else{
                rightThrottle = 1;
                leftThrottle = 1;
            }
        }
        else {
            switch (type){
                case SPLINE:
                    spinPower = 1;
                    rightThrottle = splinemath.returnPowerR(posSystem.getMotorClicks()[2], posSystem.getMotorClicks()[0]);
                    leftThrottle = splinemath.returnPowerL(posSystem.getMotorClicks()[2], posSystem.getMotorClicks()[0]);
                    //NOTE: May need to apply the powers to the spin power instead of the throttles (and make it spinPowerR, spinPowerL)
                    break;

                case LINEAR:
                    spinPower = linearmath.getSpinPower(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
                    break;
            }
        }
    }

    public void setCurrents(){
        currentW = posSystem.getPositionArr()[2];
        currentR = posSystem.getPositionArr()[3];
    }

    public void setMode(Mode mode){
        this.mode = mode;
    } //Important to set this at the "init" for teleop and auto

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

    public double[] getRobotDirection(double targetOrientation){
        double[] directionArr = new double[3];

        //determine targets
        directionArr[1] = targetOrientation;

        //determine how much robot header must turn in which direction
        double turnAmount = targetOrientation-currentR;
        double switchMotors = Math.signum(turnAmount);
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = switchMotors;

        return directionArr;
    }

    public DriveType getDriveType(){
        return type;
    }

    public double[] getVelocity(){
        double[] motorPower = new double[4];

        motorPower[0] = constants.MAX_VELOCITY_DT * speed * leftThrottle * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top left
        motorPower[1] = constants.MAX_VELOCITY_DT * speed * leftThrottle * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom left
        motorPower[2] = constants.MAX_VELOCITY_DT * speed * rightThrottle * (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //top right
        motorPower[3] = constants.MAX_VELOCITY_DT * speed * rightThrottle * (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors); //bottom right

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

            resetWheelPID.setTargets(0, 0, 0, 0);
            rotatePower = resetWheelPID.update(currentW);

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
        finished_stop = false;
        finished_snap = false;
        dontspline = false;
        firstMovement = true;

        return (currentW == 0);
    }
}

