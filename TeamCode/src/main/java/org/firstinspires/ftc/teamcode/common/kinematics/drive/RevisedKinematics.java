package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

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
    public int rightThrottle = -1;
    public int leftThrottle = -1;

    public double target = 0;
    public double turnAmountL = 0;
    public double turnAmountR = 0;

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    public Accelerator accelerator;
    TrackJoystick joystickTracker;

    //PIDs
    protected SnapSwerveModulePID snapLeftWheelPID;
    protected SnapSwerveModulePID snapRightWheelPID;

    public boolean firstMovement = true;

    double[] motorPower = new double[4];

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();

        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
        snapRightWheelPID.setTargets(0.03, 0, 0.01);

        accelerator = new Accelerator();
        joystickTracker = new TrackJoystick();
    }

    public void logic(double lx, double ly, double rx, double ry, boolean allignedWheels){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
        joystickTracker.trackJoystickL(lx, ly);

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

        firstMovement();
        turn(allignedWheels);
    }

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 ) firstMovement = true;
        if (firstMovement){
            if (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE){
                translatePerc = 0;
                rotatePerc = 1;
                spinPower = 0;
                spinClicksL = 0;
                spinClicksR = 0;
            } else{
                firstMovement = false;
                translatePerc = 0.6;
                rotatePerc = 0.4;
            }
        }
        if (noMovementRequests()) firstMovement = true;
    }

    public void turn(boolean wheelsAlligned){
        if (wheelsAlligned){
            spinClicksL = (int) (rx * 100 );
            spinClicksR = (int) (-rx * 100);
            spinPower = rx;

            leftRotClicks = 0;
            leftRotatePower = 0;
            rightRotClicks = 0;
            rightRotatePower = 0;

            translatePerc = 1;
            rotatePerc = 0;
        }
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

    public double[] getPower(){
        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right

//        double accelerationFactor = accelerator.update(1.0);
//
//        for (int i = 0; i < 4; i++){
//            motorPower[i] *= accelerationFactor;
//        }
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
    public double getLTurnAmount(){
        return turnAmountL;
    }
    public double getRTurnAmount(){
        return turnAmountR;
    }


    public void switchSpinDirection(){
        leftThrottle *= -1;
        rightThrottle *= -1;
    }
}

