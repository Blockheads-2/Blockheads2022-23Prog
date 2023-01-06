package org.firstinspires.ftc.teamcode.common.kinematics;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.ArmAuto;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

import java.util.HashMap;

public class RevisedKinematics {
    protected Constants constants = new Constants();

    private double lx;
    private double ly;
    private double rx;
    private double ry;
    double rt;
    double lt;

    double targetX;
    double targetY;
    private double finalAngle; //auto
    private double speed;
    public double distanceL;
    public double distanceR;
    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();
    TurnMath turnMath = new TurnMath();
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

    //robot's power
    private double power = 0.0;

    public double telSpinPowerL = 0.0;
    public double telSpinPowerR = 0.0;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicksR = 0; //make protected later
    public int spinClicksL = 0; //make protected later
    public double rightThrottle = 1;
    public double leftThrottle = -1;
    public double target = 0;
    public double turnAmountL = 0;
    public double turnAmountR = 0;

    //current orientation
    GlobalPosSystem posSystem;
    double currentX;
    double currentY;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    public Accelerator accelerator;
    TrackJoystick joystickTracker;

    //PIDs
    SnapSwerveModulePID snapLeftWheelPID;
    SnapSwerveModulePID snapRightWheelPID;

    public double kp = 0.03;
    public double ki = 0;
    public double kd = 0.01;

    public boolean firstMovement = true;

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();

        snapLeftWheelPID.setTargets(constants.kp, constants.ki, constants.kd);
        snapRightWheelPID.setTargets(constants.kp, constants.ki, constants.kd);

        accelerator = new Accelerator();
        joystickTracker = new TrackJoystick();

        rightThrottle = constants.initDirectionRight;
        leftThrottle = constants.initDirectionLeft;
    }

    public void logic(double lx, double ly, double rx, double ry, double rt, double lt){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
        this.rt = rt;
        this.lt = lt;

        //tracking the joystick's movement
        joystickTracker.trackJoystickL(lx, ly);

        if (noMovementRequests()) type = DriveType.STOP;
        else type = DriveType.LINEAR;

        //unnecessary function but useful for telemetry
        if (type == DriveType.STOP){
            stop();
            return;
        }

        //determining current position
        currentX = posSystem.getPositionArr()[0];
        currentY = posSystem.getPositionArr()[1];
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        //determining targets, and how much we want to turn
        target = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) target = 0;
        else if (lx==0 && ly < 0) target=180;

//        turnAmountL = wheelOptimization(target, leftCurrentW);
//        turnAmountR = wheelOptimization(target, rightCurrentW);
        turnAmountL = wheelOptimization(target, leftCurrentW, false);
        turnAmountR = wheelOptimization(target, rightCurrentW, true);
//        turnAmountL = fieldCentricWheelOptimization(target, leftCurrentW, false);
//        turnAmountR = fieldCentricWheelOptimization(target, rightCurrentW, true);

        //determining spin power
        power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        spinClicksL = (int)(power * 100 * Math.signum(leftThrottle));
        spinClicksR = (int)(power * 100 * Math.signum(rightThrottle));

        //determining rotational power
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        //determining values from right stick input.
        rightStick();
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType, ArmAuto.ArmType armType){ //runs once
        posSystem.resetXY();
        this.type = driveType;

        //target position
        this.targetX = x;
        this.targetY = y;
        this.finalAngle = finalAngle;
        this.speed = speed;

        //determining current position
        currentX = posSystem.getPositionArr()[0];
        currentY = posSystem.getPositionArr()[1];
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        linearMath.setInits(this.currentX, this.currentY);
        linearMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd);

        splineMath.setInits(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);
        splineMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd);

        firstMovement = (type == DriveType.LINEAR);
    }

    public void logicAuto(){ //runs once
        currentX = posSystem.getPositionArr()[0];
        currentY = posSystem.getPositionArr()[1];
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        if (type == DriveType.STOP) {
            stop();
            return;
        }
        //2) determining targets
        target = Math.toDegrees(Math.atan2(targetX, targetY));
        if (targetX == 0 && targetY == 0) target = 0;
        else if (targetX == 0 && targetY < 0) target = 180;

        //3) determining rotation amount
        turnAmountL = wheelOptimization(target, leftCurrentW, false);
        turnAmountR = wheelOptimization(target, rightCurrentW, true);

        //4) determining distance travel amount
        switch(type){
            case LINEAR:
                distanceR = linearMath.distanceRemaining(0, 0);
                distanceL = distanceR;
                turnAmountL = 0;
                turnAmountR = 0;
                break;

            case CONSTANT_SPLINE:
                distanceR = linearMath.distanceRemaining(0, 0);
                distanceL = distanceR;
                break;

            case VARIABLE_SPLINE:
                distanceL = splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[0];
                distanceR = splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[1];
                turnAmountL = 0;
                turnAmountR = 0;
                break;

            case TURN:
                distanceL = turnMath.getDistanceLeft(posSystem.getMotorClicks()[2]);
                distanceR = -distanceL;
                turnAmountL = 0;
                turnAmountR = 0;
                break;

            case SNAP:
                distanceL = 0;
                distanceR = 0;
                break;

            case STOP:
                distanceL = 0;
                distanceR = 0;
                turnAmountL = 0;
                turnAmountR = 0;
                break;
        }

        //determining spin power
        power = speed;
        spinClicksL = (int)(distanceL * constants.CLICKS_PER_INCH * Math.signum(leftThrottle));
        spinClicksR = (int)(distanceR * constants.CLICKS_PER_INCH * Math.signum(rightThrottle));

        //determining rotational power
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
    }

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;
        if (firstMovement){
            if (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE){
                spinClicksL = 0;
                spinClicksR = 0;
            } else{
                firstMovement = false;
            }
        }
    }

    public void rightStick(){
        if (Math.abs(leftCurrentW) < constants.degreeTOLERANCE && Math.abs(rightCurrentW) < constants.degreeTOLERANCE){
            if ((lx == 0 && ly == 0) && (rx != 0 || ry != 0)){
                //            leftThrottle = leftThrottle;
                rightThrottle *= -1;

                spinClicksL = (int) (rx * 100 * leftThrottle);
                spinClicksR = (int) (rx * 100 * rightThrottle);
                power = rx;

                turnAmountL = -leftCurrentW;
                leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);

                turnAmountR = -rightCurrentW;
                rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

                type = DriveType.TURN;
            } else if ((lx != 0 || ly != 0) && (rx != 0 && ry == 0)){
                double throttle = (ry <= lx ? ry / (1.5*rx) : rx / (1.5 * ry));
                throttle = Math.abs(throttle);
                if (rx < 0) leftThrottle *= throttle;
                else rightThrottle *= throttle;
                type = DriveType.VARIABLE_SPLINE;

            } else if ((lx == 0 && ly == 0 && rx == 0 && ry == 0) && (rt != 0 || lt != 0)){
                leftThrottle *= 0.3;
                rightThrottle *= -0.3;

                double trigger = (Math.abs(rt) > Math.abs(lt) ? rt : lt);
                spinClicksL = (int) (trigger * 100 * leftThrottle);
                spinClicksR = (int) (trigger * 100 * rightThrottle);
                power = trigger;

                turnAmountL = -leftCurrentW;
                leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);

                turnAmountR = -rightCurrentW;
                rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

                type = DriveType.TURN;
            }
        }
    }

    public void stop(){
        target = 0;
        turnAmountL = 0;
        turnAmountR = 0;

        power = 0;
        spinClicksR = 0;
        spinClicksL = 0;

        rightRotClicks = 0;
        leftRotClicks = 0;
    }

//    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
//        double target2 = (target < 0 ? target + 360 : target);
//        double current2 = (currentW < 0 ? currentW + 360 : currentW);
//
//        double turnAmount1 = target - currentW;
//        double turnAmount2 = target2 - current2;
//        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
//
//        rightThrottle = constants.initDirectionRight;
//        leftThrottle = constants.initDirectionLeft;
//
//        return turnAmount;
//    }

    public double wheelOptimization(double target, double currentW, boolean right){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if (right) rightThrottle = constants.initDirectionRight;
        else leftThrottle = constants.initDirectionLeft;

        if(Math.abs(turnAmount) > 90){
            double temp_target = clamp(target + 180);
            turnAmount = temp_target - currentW;

            if (right) this.rightThrottle *= -1;
            else this.leftThrottle *= -1;
        }
        return turnAmount;
    }

    public double fieldCentricWheelOptimization(double target, double currentW, boolean right){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW - currentR;
        double turnAmount2 = target2 - current2 - currentR;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if (right) rightThrottle = constants.initDirectionRight;
        else leftThrottle = constants.initDirectionLeft;

        if(Math.abs(turnAmount) > 90){
            double temp_target = clamp(target + 180);
            turnAmount = temp_target - currentW;

            if (right) this.rightThrottle *= -1;
            else this.leftThrottle *= -1;
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

    public double clampConventional(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < 0){
            degrees = 360 - Math.abs(degrees);
        } else if (degrees >= 360){
            degrees = Math.abs(degrees) - 360;
        }
        return degrees;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];
        motorPower[0] = (power * leftThrottle); //top left
        motorPower[1] = (power * leftThrottle); //bottom left
        motorPower[2] = (power * rightThrottle); //top right
        motorPower[3] = (power * rightThrottle); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
            motorPower[i] *= constants.POWER_LIMITER;

            if (motorPower[i] > constants.POWER_LIMITER) motorPower[i] = constants.POWER_LIMITER;
            else if (motorPower[i] < -constants.POWER_LIMITER) motorPower[i] = -constants.POWER_LIMITER;
        }
        telSpinPowerR = accelerator.update(power * constants.POWER_LIMITER * rightThrottle);
        telSpinPowerL = accelerator.update(power * constants.POWER_LIMITER * leftThrottle);

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


    public void switchLeftSpinDirection(){
        leftThrottle *= -1;
    }

    public void switchRightSpinDirection(){
        rightThrottle *= -1;
    }
}