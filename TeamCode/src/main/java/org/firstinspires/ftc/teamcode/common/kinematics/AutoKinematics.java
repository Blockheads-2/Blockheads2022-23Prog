package org.firstinspires.ftc.teamcode.common.kinematics;

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
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

import java.util.*;


import java.util.HashMap;

public class AutoKinematics {
    Constants constants = new Constants();
    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();
    TurnMath turnMath = new TurnMath();

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
    AutoPID spinPID = new AutoPID();

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

        //consider creating a nested hashmap.
        //HashMap<String, HashMap<String, Double>> params = new HashMap<String, HashMap<String, Double>>();

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

        params.put("distanceR", 0.0);
        params.put("distanceL", 0.0);

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

        params.put("X", posSystem.getPositionArr()[0]);
        params.put("Y", posSystem.getPositionArr()[1]);
        params.put("WL", posSystem.getPositionArr()[2]);
        params.put("WR", posSystem.getPositionArr()[3]);
        params.put("R", posSystem.getPositionArr()[4]);

        spinPID.setTargets(x, y, 0.3, 0, 0.1);

        switch(type){
            case LINEAR: //firstMovement true
                firstMovement = true;
                linearMath.setPos(x, y, finalAngle, 0.3, 0, 0.1);
                linearMath.setInits(params.get("X"), params.get("Y"));

                break;

            case CONSTANT_SPLINE: //firstMovement true
                firstMovement = false;
                linearMath.setPos(x, y, 0, 0.3, 0, 0.1);
                linearMath.setInits(params.get("X"), params.get("Y"));

                break;

            case VARIABLE_SPLINE:
                firstMovement = false;
                splineMath.setPos(x, y, finalAngle, 0.3, 0, 0.1);
                splineMath.setInits(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);

                break;

            case TURN:
                firstMovement = false;
                turnMath.setPos(finalAngle, 0.3, 0, 0.1, posSystem.getPositionArr()[2]);

                break;

            case STOP:
                firstMovement = false;

                break;
        }
    }

    public void logic(){
        //1) determining current position
        params.put("X", posSystem.getPositionArr()[0]);
        params.put("Y", posSystem.getPositionArr()[1]);
        params.put("WL", posSystem.getPositionArr()[2]);
        params.put("WR", posSystem.getPositionArr()[3]);
        params.put("R", posSystem.getPositionArr()[4]);

        //sidenote: if we're supposed to stop, we can ignore everything.
        if (type == DriveType.STOP) {
            stop();
            return;
        }

        //2) determining targets
        double target = Math.toDegrees(Math.atan2(input.get("X"), input.get("Y")));
        if (input.get("X") == 0 && input.get("Y") == 0) target = 0;
        else if (input.get("X")==0 && input.get("Y") < 0) target=180;

        //3) determining rotation amount
        params.put("turnAmountL", wheelOptimization(target, params.get("WL")));
        params.put("turnAmountR", wheelOptimization(target, params.get("WR")));

        //determining distance left
        switch(type){
            case LINEAR: //firstMovement true
                params.put("distanceR", linearMath.distanceRemaining(params.get("X"), params.get("Y")));
                params.put("distanceL", linearMath.distanceRemaining(params.get("X"), params.get("Y")));
                break;

            case CONSTANT_SPLINE: //firstMovement true
                params.put("distanceR", linearMath.distanceRemaining(params.get("X"), params.get("Y")));
                params.put("distanceL", linearMath.distanceRemaining(params.get("X"), params.get("Y")));
                break;

            case VARIABLE_SPLINE:
                params.put("distanceL", splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[0]);
                params.put("distanceR", splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[1]);

                break;

            case TURN:
                params.put("distanceL", turnMath.getDistanceLeft(posSystem.getMotorClicks()[2]));
                params.put("distanceR", -params.get("distanceL"));
                break;

            default:
                params.put("distanceR", 0.0);
                params.put("distanceL", 0.0);
        }

        //determining spin power
        output.put("spinPower", spinPID.update(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]));
        output.put("spinClicksL", params.get("distanceL") * constants.CLICKS_PER_INCH * Math.signum(params.get("throttleL")));
        output.put("spinClicksR", params.get("distanceR") * constants.CLICKS_PER_INCH * Math.signum(params.get("throttleR")));

        //determining rotational power
        output.put("rotPowerL", snapLeftWheelPID.update(params.get("turnAmountL")));
        output.put("rotClicksL", (params.get("turnAmountL") * constants.CLICKS_PER_DEGREE));
        output.put("rotPowerR", snapLeftWheelPID.update(params.get("turnAmountR")));
        output.put("rotClicksR", (params.get("turnAmountR") * constants.CLICKS_PER_DEGREE));

        //determining whether to focus more on spinning or more on rotating
        double rotatePerc = (Math.abs(params.get("turnAmountL")) + Math.abs(params.get("turnAmountR"))) / 70; //turnAmount / 35 --> percentage
        if (rotatePerc >= 0.5) rotatePerc = 0.5;
        params.put("rotatePerc", rotatePerc);
        params.put("translatePerc", 1.0 - rotatePerc);

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();
    }

    public void firstMovement(){
        if (firstMovement){
            if (Math.abs(params.get("turnAmountL")) >= constants.degreeTOLERANCE || Math.abs(params.get("turnAmountR")) >= constants.degreeTOLERANCE){
                params.put("translatePerc", 0.0);
                params.put("rotatePerc", 1.0);

                output.put("spinPowerL", 0.0);
                output.put("spinPowerR", 0.0);
                output.put("spinClicksL", 0.0);
                output.put("spinClicksR", 0.0);
            } else{
                firstMovement = false;
                params.put("translatePerc", 0.6);
                params.put("rotatePerc", 0.4);
            }
        }
    }

    public void stop(){
        for (Map.Entry<String,Double> mapElement : output.entrySet()) { //iterates through each output value and sets them to 0.
            String key = mapElement.getKey();
            output.put(key, 0.0);
        }
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        params.put("throttleR", 1.0);
        params.put("throttleL", -1.0);

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
        motorPower[0] = output.get("spinPowerL") * params.get("translatePerc") * params.get("throttleL") + output.get("rotPowerL") * params.get("rotatePerc"); //top left
        motorPower[1] = output.get("spinPowerL") * params.get("translatePerc") * params.get("throttleL") + output.get("rotPowerL") * params.get("rotatePerc"); //bottom left
        motorPower[2] = output.get("spinPowerR") * params.get("translatePerc") * params.get("throttleR") + output.get("rotPowerR") * params.get("rotatePerc"); //top right
        motorPower[3] = output.get("spinPowerR") * params.get("translatePerc") * params.get("throttleR") + output.get("rotPowerR") * params.get("rotatePerc"); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
        }
        return motorPower;
    }


    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = (int)(output.get("spinClicksL") + output.get("rotClicksL")); //left
        clicks[1] = (int)(-output.get("spinClicksL") + output.get("rotClicksL")); //left
        clicks[2] = (int)(output.get("spinClicksR") + output.get("rotClicksR")); //right
        clicks[3] = (int)(-output.get("spinClicksR") + output.get("rotClicksR")); //right
        return clicks;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean isFinishedMovement(){
        return (finishedMovement);
    }

    public double getTarget(){
        double target = Math.toDegrees(Math.atan2(input.get("X"), input.get("Y")));
        if (input.get("X") == 0 && input.get("Y") == 0) target = 0;
        else if (input.get("X")==0 && input.get("Y") < 0) target=180;
        return  target;
    }

    public void switchSpinDirection(){
        params.put("throttleR", params.get("throttleR") * -1);
        params.put("throttleL", params.get("throttleL") * -1);
    }
}

