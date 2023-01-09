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
    GlobalPosSystem posSystem;

//    public enum Side{
//        RIGHT,
//        LEFT
//    }
//    Side side;


    //teleop
    private double currentW = 0;
    private double nonRightStickCurrentW = 0;

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

    public SwervePod(double spinDirection, GlobalPosSystem gps){
        pid = new SnapSwerveModulePID();
        this.direction = (int)spinDirection;
        this.initDirection = this.direction;
        this.posSystem = gps;
    }

    public void setPID(double kp, double ki, double kd){
        pid.setTargets(kp, ki, kd);
    }

    public void setPower(double power){
        this.power = power;
    }

    public void setPowerUsingLeft(SwervePod PodR){
        power = (driveType == RevisedKinematics.DriveType.SNAP ?
                ((Math.abs(pid.update(turnAmount)) + Math.abs(PodR.getPID().update(PodR.getTurnAmount()))) / 2.0) * speed :
                ((Math.abs(pid.update(distance)) + Math.abs(PodR.getPID().update(PodR.getDistance()))) / 2.0) * speed);
    }

    public void setCurrents(double currentW, boolean rightStick){
        this.currentW = currentW;

        if (!rightStick) nonRightStickCurrentW = currentW;
    }

    public void setRotClicks(double target){
        turnAmount = wheelOptimization(target, currentW);

        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

    public RevisedKinematics.DriveType setSpinClicks(double powerFactor, double trigger, boolean turn, boolean spline, double rightStickX, double rightStickY, boolean right){
        spinClicksTarget = (int)(power * (100 * (1.0 + trigger)) * direction);
        throttle = 1.0;

        if (turn){
            if (right) direction *= -1;

            spinClicksTarget = rightStickX * 100 * direction;
            power = rightStickX;

            setRotClicks(nonRightStickCurrentW);

            return RevisedKinematics.DriveType.TURN;
        } else if (spline){
            double throttle = (rightStickY <= rightStickX ? rightStickY / (1.5*rightStickX) : rightStickX / (1.5 * rightStickY));
            throttle = Math.abs(throttle);
            if (rightStickX < 0) {
                if (!right) this.throttle *= throttle;
            }
            else {
                if (right) this.throttle *= throttle;
            }
            return RevisedKinematics.DriveType.VARIABLE_SPLINE;
        }

        return RevisedKinematics.DriveType.LINEAR;
    }

    public void setSpinClicks(int clicks) {
        spinClicksTarget = (double) (clicks);
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, RevisedKinematics.DriveType driveType){
        this.driveType = driveType;

        //target position
        this.speed = speed;
        this.finalAngle = finalAngle;

        linearMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);

        splineMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd, posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);

        turnMath.setPos(finalAngle, posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2], (finalAngle < 0 ? -1 : 1));

        setPID(constants.kp, constants.ki, constants.kd);

        power = (driveType == RevisedKinematics.DriveType.SNAP ? speed * pid.update(turnAmount) : speed * pid.update(distance));
    }

    public void autoLogic(double currentW, boolean rightStick, boolean right){
        setCurrents(currentW, rightStick);

        //determining distance left for rotating and spinning the module
        switch(driveType){
            case CONSTANT_SPLINE:

            case LINEAR:
                if (!right) distance = linearMath.distanceRemainingL(posSystem.getMotorClicks()[0]);
                else distance = linearMath.distanceRemainingR(posSystem.getMotorClicks()[2]);
//                distance = (linearMath.distanceRemainingL(posSystem.getMotorClicks()[0]) + linearMath.distanceRemainingR(posSystem.getMotorClicks()[2])) / 2.0;

                throttle = 1.0;

                setRotClicks(finalAngle);
                break;

            case VARIABLE_SPLINE: //using gps for variable_spline may be incredibly unreliable.  Though the nature of clicks (they are integers) gives us an inaccurate account (but this fear depends on the fact that the code loops really really quickly).
                if (!right) distance = splineMath.distanceRemainingL(posSystem.getMotorClicks()[0]);
                else distance = splineMath.distanceRemainingR(posSystem.getMotorClicks()[2]);
                setRotClicks(nonRightStickCurrentW);
                break;

            case SNAP:
                distance = 0;
                setRotClicks(finalAngle);
                throttle = 1.0;

                break;

            case TURN:
                if (!right) distance = turnMath.distanceRemainingL(posSystem.getMotorClicks()[0]);
                else distance = turnMath.distanceRemainingR(posSystem.getMotorClicks()[2]);
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

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        direction = initDirection;

        if(Math.abs(turnAmount) > 90){
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
}
