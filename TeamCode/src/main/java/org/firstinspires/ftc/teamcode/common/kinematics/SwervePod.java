package org.firstinspires.ftc.teamcode.common.kinematics;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.HeaderControlPID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

import java.util.HashMap;

public class SwervePod {
    Constants constants = new Constants();
    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();
    TurnMath turnMath = new TurnMath();
    public HeaderControlPID controlHeader;
    Accelerator accelerator;

    public enum Side{
        RIGHT,
        LEFT
    }
    Side side;

    //teleop
    private double currentW = 0;
    private double currentR = 0;
    public double optimizedCurrentW = 0;
    public double controlHeaderReference = 0;
    public double nonRightStickCurrentW = 0;
    public boolean initPole = true;

    private double throttle = 0;
    public int direction;
    private int initDirection;
    private double turnAmount = 0;

    private double power = 0;
    public double spinClicksTarget = 0;
    public double rotClicksTarget = 0;

    //auto
    private double distance = 0;
    private double speed = 0;
    private double finalAngle = 0;
    private RevisedKinematics.DriveType driveType = RevisedKinematics.DriveType.NOT_INITIALIZED;

    //both teleop & auto
    SnapSwerveModulePID pid;

    private double[] output = new double[3];

    public SwervePod(int spinDirection, Side side, Accelerator accelerator){
        pid = new SnapSwerveModulePID();
        this.direction = spinDirection;
        this.initDirection = spinDirection;
        this.side = side;
        this.accelerator = accelerator;
        this.controlHeader = controlHeader;
    }

    public void setHeaderController(HeaderControlPID controlHeader){
        this.controlHeader = controlHeader;
    }


    public void setPID(double kp, double ki, double kd){
        pid.setTargets(kp, ki, kd);
    }

    public void setCurrents(double currentW, double currentR){
        this.currentW = currentW;
        this.currentR = currentR;
    }

    public void setRotClicks(double target){
        turnAmount = wheelOptimization(target, currentW);
        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

    public void forceSetRotClicks(int clicks){
        rotClicksTarget = clicks;
    }

    //for spin power and spin clicks
    public RevisedKinematics.DriveType setSpinClicksAndPower(double powerFactor, double trigger, boolean turn, boolean spline, boolean eligibleForTurning, double rightStickX, int[] pos){ //teleop
        this.power = powerFactor;
        this.spinClicksTarget = (power * (constants.SPIN_CLICK_FACTOR * (1.0 + (trigger))));
        this.throttle = 1.0;

        if (turn) {
            if (eligibleForTurning) {
                setRotClicks(currentR);
                //            leftThrottle = leftThrottle;
                if (rightStickX < 0 && side == Side.LEFT) direction *= -1;
                else if (rightStickX >= 0 && side == Side.RIGHT) direction *= -1;

                this.spinClicksTarget = Math.abs(rightStickX) * constants.SPIN_CLICK_FACTOR;
                power = rightStickX;
            } else {
                setRotClicks(currentR);
                spinClicksTarget = 0;
                power = 1.0;
            }

//            controlHeader.calculateThrottle(pos, currentR, controlHeaderReference, true);
//            controlHeaderReference = currentR;

            driveType = RevisedKinematics.DriveType.TURN;
            return driveType;
        } else if (spline){
//            double throttle = (rightStickY <= rightStickX ? rightStickY / (1.5*rightStickX) : rightStickX / (1.5 * rightStickY));
//            double throttle = Math.abs(Math.tanh(rightStickX));
            double throttle = 1.0 - Math.sin(Math.abs(rightStickX));
            if (rightStickX < 0 && side == Side.LEFT) this.throttle = throttle;
            else if (rightStickX >= 0 && side == Side.RIGHT) this.throttle = throttle;

//            if (SwervePod.changeAngle(controlHeaderReference, currentR) > 30) controlHeaderReference = currentR;
//            controlHeaderReference = currentR;
//            controlHeader.calculateThrottle(pos, currentR, controlHeaderReference, true);

            driveType = RevisedKinematics.DriveType.VARIABLE_SPLINE;
            return driveType;
        } else {
            nonRightStickCurrentW = optimizedCurrentW;
            direction = (initPole ? initDirection : -initDirection);
//            if (SwervePod.changeAngle(controlHeaderReference, currentR) > 30) controlHeaderReference = currentR;
//            controlHeader.calculateThrottle(pos, currentR, controlHeaderReference, false);
//            throttle = controlHeader.getThrottle(side);
        }

        if (rightStickX == 0 && power == 0) {
            driveType = RevisedKinematics.DriveType.STOP;
            return driveType;
        }

        driveType = RevisedKinematics.DriveType.LINEAR;
        return driveType;
    }

    public double wheelOptimization(double target, double currentW){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - clamp(currentW + (initPole ? 0 : 180));
        double turnAmount2 = target2 - clampConventional(current2 + (initPole ? 0 : 180));

        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if(Math.abs(turnAmount) > 90){
            initPole = !initPole;
            double temp_target = clamp(target + 180);
            turnAmount = changeAngle(temp_target, currentW);
        }

        if (initPole){
            direction = initDirection;
            optimizedCurrentW = currentW;
        } else{
            direction = -initDirection;
            optimizedCurrentW = clamp(currentW + 180);
        }


        return turnAmount;
    }


    public void setSpinClicks(int clicks) {
        spinClicksTarget = (double) (clicks);
    }

    public void setThrottle(double throttle){
        this.throttle = throttle;
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, RevisedKinematics.DriveType driveType, int initClicks, boolean right, int[] posClicks, double currentW, double currentR){
        setCurrents(currentW, currentR);

        this.driveType = driveType;

        if (this.driveType != RevisedKinematics.DriveType.SNAP) finalAngle = currentW;

        //target position
        this.speed = speed;
        this.finalAngle = finalAngle;

        linearMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, initClicks);

        splineMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, initClicks, right);

        turnMath.setPos(finalAngle, initClicks, (finalAngle < 0 ? -1 : 1), right);

        setPID(constants.kp, constants.ki, constants.kd);

        controlHeader.calculateThrottle(posClicks, currentR, currentR, true);

        setPowerAuto();
    }

    public void setPowerAuto(){ //auto
        double powerSpin = Math.abs(pid.update(distance)) * speed;
        double powerRotate = Math.abs(pid.update(turnAmount)) * speed;
        power = (driveType == RevisedKinematics.DriveType.SNAP ? powerRotate : powerSpin);
    }

    public void setPower(double power){
        this.power = power;
    }

    public void autoLogic(double currentW, double currentR, int currClick, int[] posClicks){
        setCurrents(currentW, currentR);

        if (driveType != RevisedKinematics.DriveType.TURN && driveType != RevisedKinematics.DriveType.VARIABLE_SPLINE) nonRightStickCurrentW = currentW;

        //determining distance left for rotating and spinning the module
        switch(driveType){
            case CONSTANT_SPLINE:

            case LINEAR:
                setRotClicks(finalAngle);

                distance = linearMath.distanceRemaining(currClick);
                throttle = 1.0;
                controlHeader.calculateThrottle(posClicks, currentR, currentR, true);
                direction = (initPole ? initDirection : -initDirection);

                break;

            case VARIABLE_SPLINE: //using gps for variable_spline may be incredibly unreliable.  Though the nature of clicks (they are integers) gives us an inaccurate account (but this fear depends on the fact that the code loops really really quickly).
                setRotClicks(0);

                direction = (initPole ? initDirection : -initDirection);
                distance = splineMath.distanceRemaining(currClick);
                break;

            case SNAP:
                if (side == Side.RIGHT){
                    setRotClicks(finalAngle-2);
                } else {
                    setRotClicks(finalAngle);
                }

                direction = (initPole ? initDirection : -initDirection);
                distance = 0;
                throttle = 1.0;

                break;

            case TURN:
                setRotClicks(0);

                distance = turnMath.distanceRemaining(currClick);

                direction = (initPole ? initDirection : -initDirection);
                if (finalAngle < 0 && side == Side.LEFT) direction *= -1;
                else if (finalAngle >= 0 && side == Side.RIGHT) direction *= -1;

                break;

            case STOP:
                distance = 0;
                turnAmount = 0;
                throttle = 1.0;
                break;
        }

        spinClicksTarget = distance * constants.CLICKS_PER_INCH;
        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;

        setPowerAuto();
    }

    public static double changeAngle(double target, double current){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (current < 0 ? current + 360 : current);
        double turnAmount1 = target - current;
        double turnAmount2 = target2 - current2;
        return (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
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

    public boolean onlyRotate(boolean firstMovement){
        if (firstMovement){
            return (Math.abs(turnAmount) >= constants.degreeTOLERANCE);
        }
        return false;
    }

    public RevisedKinematics.DriveType getDriveType(){
        return driveType;
    }

    public double[] getOutput(){
        power = accelerator.update(Math.abs(power) * constants.POWER_LIMITER * direction, turnAmount);

        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        throttle = Math.abs(throttle);
        power *= throttle;
        spinClicksTarget = Math.abs(spinClicksTarget) * direction * throttle;

        output[0] = spinClicksTarget;
        output[1] = rotClicksTarget;
        output[2] = power;

        return output;
    }

    public double[] getOutputAuto(){
        power = accelerator.update(Math.abs(power) * constants.POWER_LIMITER * direction, turnAmount);

        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        throttle = Math.abs(throttle);
        power *= throttle;
        spinClicksTarget = spinClicksTarget * direction * throttle;

        output[0] = spinClicksTarget;
        output[1] = rotClicksTarget;
        output[2] = power;

        return output;
    }

    public int getSpinDirection(){
        return direction;
    }

    public SnapSwerveModulePID getPID(){
        return pid;
    }

    public double getTurnAmount(){
        return turnAmount;
    }

    public double getDistance(){
        return distance;
    }

    public double getThrottle(){
        return throttle;
    }
}
