package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
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

    public enum Side{
        RIGHT,
        LEFT
    }
    Side side;


    //teleop
    private double currentW = 0;
    private double nonRightStickCurrentW = 0;
    private boolean initPole = true;

    private double throttle = 0;
    private int direction;
    private int initDirection;
    private double turnAmount = 0;

    private double power = 0;
    private double spinClicksTarget = 0;
    private double rotClicksTarget = 0;

    //auto
    private double distance = 0;
    private double speed = 0;
    private double finalAngle = 0;
    private RevisedKinematics.DriveType driveType = RevisedKinematics.DriveType.NOT_INITIALIZED;

    //both teleop & auto
    SnapSwerveModulePID pid;

    public HashMap<String, Double> output = new HashMap<String, Double>();

    public SwervePod(double spinDirection, Side side){
        pid = new SnapSwerveModulePID();
        this.direction = (int)spinDirection;
        this.initDirection = this.direction;
        this.side = side;
    }

    public void setPID(double kp, double ki, double kd){
        pid.setTargets(kp, ki, kd);
    }

    public void setPowerUsingLeft(SwervePod PodR){ //auto
        power = (driveType == RevisedKinematics.DriveType.SNAP ?
                ((Math.abs(pid.update(turnAmount)) + Math.abs(PodR.getPID().update(PodR.getTurnAmount()))) / 2.0) * speed :
                ((Math.abs(pid.update(distance)) + Math.abs(PodR.getPID().update(PodR.getDistance()))) / 2.0) * speed);
    }

    public void setCurrents(double currentW){
        this.currentW = currentW;
    }

    public void setRotClicks(double target){
        turnAmount = wheelOptimization(target, currentW);

        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

    public RevisedKinematics.DriveType setSpinClicks(double powerFactor, double trigger, boolean turn, boolean spline, double rightStickX){ //teleop
        this.power = powerFactor;
        spinClicksTarget = (int)(power * (100 * (1.0 + trigger)) * direction);
        throttle = 1.0;

        if (turn){
            if (side == Side.RIGHT) direction *= -1;

            spinClicksTarget = rightStickX * 100 * direction;
            power = rightStickX;

            setRotClicks(nonRightStickCurrentW);

            return RevisedKinematics.DriveType.TURN;
        } else if (spline){
//            double throttle = (rightStickY <= rightStickX ? rightStickY / (1.5*rightStickX) : rightStickX / (1.5 * rightStickY));
            double throttle = Math.sin(rightStickX);
            throttle = Math.abs(throttle);
            if (rightStickX < 0 && side != Side.RIGHT) this.throttle *= throttle;
            else if (rightStickX >= 0 && side == Side.RIGHT) this.throttle *= throttle;

            return RevisedKinematics.DriveType.VARIABLE_SPLINE;
        }

        if (!turn && !spline) nonRightStickCurrentW = currentW;

        if (powerFactor == 0) return RevisedKinematics.DriveType.STOP;

        if (throttle > 1.0) throttle = 1;
        else if (throttle < 0) throttle = 0;

        return RevisedKinematics.DriveType.LINEAR;
    }

    public void setThrottleUsingPodLReference(SwervePod PodR, boolean turn, boolean spline){
        double minThrottle = Math.min(PodR.getThrottle(), getThrottle());
        minThrottle += 0.1; //play with the 0.1 factor
        if (!turn && !spline){
            if (minThrottle > 1.0) minThrottle = 1;
            else if (minThrottle < 0) minThrottle = 0;

            PodR.setThrottle(minThrottle);
            setThrottle(minThrottle);
        }
    }

    public void setSpinClicks(int clicks) {
        spinClicksTarget = (double) (clicks);
    }

    public void setThrottle(double throttle){
        this.throttle = throttle;
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, RevisedKinematics.DriveType driveType, int initClicks, boolean right){
        this.driveType = driveType;

        //target position
        this.speed = speed;
        this.finalAngle = finalAngle;

        linearMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, initClicks);

        splineMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, initClicks, right);

        turnMath.setPos(finalAngle, initClicks, (finalAngle < 0 ? -1 : 1), right);

        setPID(constants.kp, constants.ki, constants.kd);

        power = (driveType == RevisedKinematics.DriveType.SNAP ? speed * pid.update(turnAmount) : speed * pid.update(distance));
    }

    public void autoLogic(double currentW, int currClick){
        setCurrents(currentW);

        if (driveType != RevisedKinematics.DriveType.TURN && driveType != RevisedKinematics.DriveType.VARIABLE_SPLINE) nonRightStickCurrentW = currentW;

        //determining distance left for rotating and spinning the module
        switch(driveType){
            case CONSTANT_SPLINE:

            case LINEAR:
                distance = linearMath.distanceRemaining(currClick);
                throttle = 1.0;

                setRotClicks(finalAngle);
                break;

            case VARIABLE_SPLINE: //using gps for variable_spline may be incredibly unreliable.  Though the nature of clicks (they are integers) gives us an inaccurate account (but this fear depends on the fact that the code loops really really quickly).
                distance = splineMath.distanceRemaining(currClick);
                setRotClicks(nonRightStickCurrentW);
                break;

            case SNAP:
                distance = 0;
                setRotClicks(finalAngle);
                throttle = 1.0;

                break;

            case TURN:
                distance = turnMath.distanceRemaining(currClick);
                setRotClicks(nonRightStickCurrentW);
                break;

            case STOP:
                distance = 0;
                turnAmount = 0;
                throttle = 1.0;
                break;
        }

        spinClicksTarget = distance * constants.CLICKS_PER_INCH;
        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

//    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
//        double target2 = (target < 0 ? target + 360 : target);
//        double current2 = (currentW < 0 ? currentW + 360 : currentW);
//
//        double turnAmount1 = target - currentW;
//        double turnAmount2 = target2 - current2;
//        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
//
//        direction = initDirection;
//
//        if(Math.abs(turnAmount) > 90){
//            double temp_target = clamp(target + 180);
//            turnAmount = temp_target - currentW;
//
//            direction *= -1;
//        }
//        return turnAmount;
//    }

    public double wheelOptimization(double target, double currentW){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - clamp(currentW + (initPole ? 0 : 180));
        double turnAmount2 = target2 - clampConventional(current2 + (initPole ? 0 : 180));

        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if(Math.abs(turnAmount) > 90){
            initPole = !initPole;

            double temp_target = clamp(target + 180);
            turnAmount = temp_target - currentW;

            direction *= -1;
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

    public boolean onlyRotate(boolean firstMovement){
        if (firstMovement){
            return (Math.abs(turnAmount) >= constants.degreeTOLERANCE);
        }
        return false;
    }

    public RevisedKinematics.DriveType getDriveType(){
        return driveType;
    }

    public void getOutput(){
        output.put("power", power);
        output.put("spinClicksTarget", spinClicksTarget);
        output.put("rotClicksTarget", rotClicksTarget);
        output.put("direction", (double)direction);
        output.put("throttle", throttle);
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
