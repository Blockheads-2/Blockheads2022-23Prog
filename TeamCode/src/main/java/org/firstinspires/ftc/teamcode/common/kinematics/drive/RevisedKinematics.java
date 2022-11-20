package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;

public class RevisedKinematics {
    protected Constants constants = new Constants();

    private double lx;
    private double ly;
    private double rx;
    private double ry;

    public enum DriveType{
        LINEAR,
        SNAP,
        STOP,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    double leftRotatePower = 0.0;
    double rightRotatePower = 0.0;
    double spinPower = 0.0;

    double translatePerc = 0.6; //placeholder for now
    double rotatePerc = 0.4;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicksR = 0; //make protected later
    public int spinClicksL = 0; //make protected later

    public int leftTurnDirectionW = 1;
    public int rightTurnDirectionW = 1;
    public int rightThrottle = -1;
    public int leftThrottle = -1;

    public double target = 0;
    public double turnAmountL = 0;
    public double turnAmountR = 0;
    private enum Module{
        RIGHT,
        LEFT
    }

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    public Accelerator accelerator;

    //PIDs
    protected SnapSwerveModulePID snapLeftWheelPID;
    protected SnapSwerveModulePID snapRightWheelPID;

    double[] motorPower = new double[4];

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();

        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
        snapRightWheelPID.setTargets(0.03, 0, 0.01);

        accelerator = new Accelerator();
    }

    public void logic(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;

        if (noMovementRequests()) type = DriveType.STOP;
        else type = DriveType.LINEAR;

        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        target = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) target = 0;
        else if (lx==0 && ly < 0) target=180;

        turnAmountL = wheelOptimization(target, leftCurrentW);
        turnAmountR = wheelOptimization(target, rightCurrentW);

        spinPower = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        spinClicksL = (int)(spinPower * 100 * leftThrottle);
        spinClicksR = (int)(spinPower * 100 * rightThrottle);

        leftRotatePower = snapLeftWheelPID.update(turnAmountL);
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotatePower = snapRightWheelPID.update(turnAmountR);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount = target - currentW;
        double turnAmount2 = target2 - current2;

        if (Math.abs(turnAmount) < Math.abs(turnAmount2)){
            return turnAmount;

        } else{
            return turnAmount2;
        }
    }

    public void wheelOptimization(double target, double currentW, Module module){ //returns how much the wheels should rotate in which direction
        double turnAmount = target - currentW;
        int turnDirection = (int)Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }

        if (target == 180){
            turnAmount = 0;
            leftThrottle = 1;
            rightThrottle = 1;
        } else{
            leftThrottle = -1;
            rightThrottle =-1;
        }

        switch (module){
            case RIGHT:
                rightTurnDirectionW = turnDirection;
                turnAmountR = Math.abs(turnAmount);
                break;

            case LEFT:
                leftTurnDirectionW = turnDirection;
                turnAmountL =  Math.abs(turnAmount);
                break;
        }
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    public double[] getPower(){
        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right

        motorPower[0] = accelerator.update(motorPower[0]);
        motorPower[1] = accelerator.update(motorPower[1]);
        motorPower[2] = accelerator.update(motorPower[2]);
        motorPower[3] = accelerator.update(motorPower[3]);

        return motorPower;
    }


    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = spinClicksL + leftRotClicks; //left
        clicks[1] = -spinClicksL + leftRotClicks; //left
        clicks[2] = spinClicksR + rightRotClicks; //right
        clicks[3] = -spinClicksR  + rightRotClicks; //right
        return clicks;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }

    public double getTarget(){
        return target;
    }
    public int getRightDirectionW(){
        return rightTurnDirectionW;
    }
    public int getLeftDirectionW(){
        return leftTurnDirectionW;
    }
    public double getLTurnAmount(){
        return turnAmountL;
    }
    public double getRTurnAmount(){
        return turnAmountR;
    }

    public void switchRotateDirection(){
        rightTurnDirectionW *= -1;
        leftTurnDirectionW *= -1;
    }

    public void switchSpinDirection(){
        leftThrottle *= -1;
        rightThrottle *= -1;
    }
}

