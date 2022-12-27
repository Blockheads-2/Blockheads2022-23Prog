package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.AutoPID;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHubJR;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;
import java.util.*;


import java.util.HashMap;

public class AutoKinematics {
    Constants constants = new Constants();
    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();

    public HashMap<String, Double> params = new HashMap<>();
    public HashMap<String, Double> input = new HashMap<>();
    public HashMap<String, Double> output = new HashMap<>();

    public enum DriveType{
        LINEAR,
        SNAP,
        STOP,
        TURN,
        CONSTANT_SPLINE,
        VARIABLE_SPLINE,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //current orientation
    GlobalPosSystem posSystem;

    public Accelerator accelerator;

    //PIDs
    SnapSwerveModulePID snapLeftWheelPID;
    SnapSwerveModulePID snapRightWheelPID;

    //check
    public boolean firstMovement = true;
    boolean finishedMovement = true;

    public AutoKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();

        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
        snapRightWheelPID.setTargets(0.03, 0, 0.01);

        accelerator = new Accelerator();

        //gps
        params.put("X", 0.0);
        params.put("Y", 0.0);
        params.put("R", 0.0);
        params.put("WL", 0.0);
        params.put("WR", 0.0);

        //throttle
        params.put("throttleR", 1.0);
        params.put("throttleL", -1.0);
        params.put("translatePerc", 0.6);
        params.put("rotatePerc", 0.4);

        //how much the module needs to rotate
        params.put("turnAmountL", 0.0);
        params.put("turnAmountR", 0.0);

        //targets
        input.put("X", 0.0);
        input.put("Y", 0.0);
        input.put("theta", 0.0);
        input.put("speed", 0.0);

        //output. Values given to the motor.
        output.put("spinPowerL", 0.0);
        output.put("spinPowerR", 0.0);
        output.put("spinClicksL", 0.0);
        output.put("spinClicksR", 0.0);

        output.put("rotPowerL", 0.0);
        output.put("rotPowerR", 0.0);
        output.put("rotClicksR", 0.0);
        output.put("rotClicksL", 0.0);
    }

    public void setDriveType(DriveType dType){
        this.type = dType;
        firstMovement = (dType == DriveType.LINEAR || dType == DriveType.STOP);
    }

    public void setPos(double x, double y, double finalAngle, double speed){
        input.put("X", x);
        input.put("Y", y);
        input.put("theta", 0.0);
        input.put("speed", speed);

        switch(type){
            case LINEAR: //firstMovement true
                firstMovement = true;
                linearMath.setPos(x, y, finalAngle, 0.3, 0, 0.01);

                break;

            case CONSTANT_SPLINE: //firstMovement true
                firstMovement = false;
                linearMath.setPos(x, y, 0, 0.3, 0, 0.01);

            case VARIABLE_SPLINE:
                splineMath.setPos(x, y, finalAngle, 0.3, 0, 0.01);

                break;

            case TURN:
                //...
                break;

            case STOP:
                stop();
                break;
        }
    }

    public void logic(){
        //1) determining current position
        params.put("WL", posSystem.getPositionArr()[2]);
        params.put("WR", posSystem.getPositionArr()[3]);
        params.put("R", posSystem.getPositionArr()[4]);

        //2) determining targets
        double target = Math.toDegrees(Math.atan2(input.get("X"), input.get("Y")));
        if (input.get("X") == 0 && input.get("Y") == 0) target = 0;
        else if (input.get("X")==0 && input.get("Y") < 0) target=180;

        //3) determining rotation amount
        params.put("turnAmountL", wheelOptimization(target, params.get("WL")));
        params.put("turnAmountR", wheelOptimization(target, params.get("WR")));

        //determining spin power
//        spinPower = spinPID.update(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
//        spinClicksL = (int)(spinPower * 100 * leftThrottle);
//        spinClicksR = (int)(spinPower * 100 * rightThrottle);

        //determining rotational power
        output.put("rotPowerL", snapLeftWheelPID.update(params.get("turnAmountL")));
        output.put("rotClicksL", (params.get("turnAmountL") * constants.CLICKS_PER_DEGREE));
        output.put("rotPowerR", snapLeftWheelPID.update(params.get("turnAmountR")));
        output.put("rotClicksR", (params.get("turnAmountR") * constants.CLICKS_PER_DEGREE));

        //determining whether to focus more on spinning or more on rotating
        double rotatePerc = (Math.abs(params.get("turnAmountL")) + Math.abs(params.get("turnAmountR"))) / 150.0; //turnAmount / 75 --> percentage
        if (rotatePerc >= 0.5) rotatePerc = 0.5;
        params.put("rotatePerc", rotatePerc);
        params.put("translatePerc", 1.0 - rotatePerc);

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        if (type == DriveType.LINEAR) firstMovement();
    }

    public void firstMovement(){
        if (firstMovement){
            if (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE){
                translatePerc = 0;
                rotatePerc = 1;
                spinPower = 0;
                spinClicksL = 0;
                spinClicksR = 0;
            } else{
                type = DriveType.LINEAR;
                translatePerc = 0.6;
                rotatePerc = 0.4;
            }
        }
    }

    public void stop(){
        output.forEach((k, v) -> {
            v=0.0;
        }); //test this later.
//        target = 0;
//        turnAmountL = 0;
//        turnAmountR = 0;
//
//        spinPower = 0;
//        spinClicksR = 0;
//        spinClicksL = 0;
//
//        rightRotatePower = 0;
//        leftRotatePower = 0;
//        rightRotClicks = 0;
//        leftRotClicks = 0;
//
//        translatePerc = 0;
//        rotatePerc = 0;
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        rightThrottle = 1;
        leftThrottle = -1;

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

    public double[] getPower(){
        double[] motorPower = new double[4];
        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
        }
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

    public boolean isFinishedMovement(){
        return (finishedMovement);
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

