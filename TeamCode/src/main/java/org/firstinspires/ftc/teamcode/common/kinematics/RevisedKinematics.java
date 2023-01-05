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

    private double finalAngle; //auto
    private double speed;
    double distanceL;
    double distanceR;
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
    private double leftRotatePower = 0.0;
    private double rightRotatePower = 0.0;
    private double spinPower = 0.0;

    public double telLeftRotatePower = 0.0;
    public double telRightRotatePower = 0.0;
    public double telSpinPowerL = 0.0;
    public double telSpinPowerR = 0.0;

    double translatePercR = 0.6; //placeholder for now
    double translatePercL = 0.6;
    double rotatePercR = 0.4;
    double rotatePercL = 0.4;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicksR = 0; //make protected later
    public int spinClicksL = 0; //make protected later
    public int rightThrottle = 1;
    public int leftThrottle = -1;
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
    AutoPID autoPID;

    public double kp = 0.3;
    public double ki = 0;
    public double kd = 0.01;

    public boolean firstMovement = true;

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();
        autoPID = new AutoPID();

        snapLeftWheelPID.setTargets(kp, ki, kd);
        snapRightWheelPID.setTargets(kp, ki, kd);

        accelerator = new Accelerator();
        joystickTracker = new TrackJoystick();

        rightThrottle = 1;
        leftThrottle = -1;
    }

    public void logic(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;

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

        //determining spin power
        spinPower = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        spinClicksL = (int)(spinPower * 100 * Math.signum(leftThrottle));
        spinClicksR = (int)(spinPower * 100 * Math.signum(rightThrottle));

        //determining rotational power
        leftRotatePower = snapLeftWheelPID.update(turnAmountL);
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotatePower = snapRightWheelPID.update(turnAmountR);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining whether to focus more on spinning or more on rotating
        rotatePercL = Math.abs(turnAmountL) / 60.0;
        if (rotatePercL > 0.5) rotatePercL = 0.5;

        rotatePercR = Math.abs(turnAmountR) / 60.0;
        if (rotatePercR > 0.5) rotatePercR = 0.5;

        translatePercL = 1.0 - rotatePercL;
        translatePercR = 1.0 - rotatePercR;

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        //determining values from right stick input.
        rightStick();
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType){ //runs once
        posSystem.resetXY();
        this.type = driveType;

        //target position
        this.finalAngle = finalAngle;
        this.speed = speed;

        //setting up PID
        autoPID.setTargets(x, y, 0.3, 0, 0.01);

        //determining current position
        currentX = posSystem.getPositionArr()[0];
        currentY = posSystem.getPositionArr()[1];
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        linearMath.setInits(this.currentX, this.currentY);
        linearMath.setPos(x, y, finalAngle, 0.3, 0, 0.01);

        splineMath.setInits(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);
        splineMath.setPos(x, y, finalAngle, 0.3, 0, 0.01);

        //turnMath.setInits

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
        double target = Math.toDegrees(Math.atan2(currentX, currentY));
        if (currentX == 0 && currentY == 0) target = 0;
        else if (currentX == 0 && currentY < 0) target = 180;

        //3) determining rotation amount
        turnAmountL = wheelOptimization(target, leftCurrentW, false);
        turnAmountR = wheelOptimization(target, rightCurrentW, true);

        //4) determining distance travel amount
        if (type == DriveType.LINEAR || type == DriveType.CONSTANT_SPLINE){
            distanceR = linearMath.distanceRemaining(currentX, currentY);
            distanceL = distanceR;
        } else if (type == DriveType.VARIABLE_SPLINE){
            distanceL = splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[0];
            distanceR = splineMath.distanceRemaining(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2])[1];
        } else if (type == DriveType.TURN){
            distanceL = turnMath.getDistanceLeft(posSystem.getMotorClicks()[2]);
            distanceR = -distanceL;
        } else {
            distanceL = 0;
            distanceR = 0;
        }

        //determining spin power
        spinPower = autoPID.update(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
        spinClicksL = (int)(distanceL * constants.CLICKS_PER_INCH * Math.signum(leftThrottle));
        spinClicksR = (int)(distanceR * constants.CLICKS_PER_INCH * Math.signum(rightThrottle));

        //determining rotational power
        leftRotatePower = snapLeftWheelPID.update(turnAmountL);
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotatePower = snapRightWheelPID.update(turnAmountR);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining whether to focus more on spinning or more on rotating
        rotatePercL = Math.abs(turnAmountL) / 60.0;
        if (rotatePercL > 0.5) rotatePercL = 0.5;

        rotatePercR = Math.abs(turnAmountR) / 60.0;
        if (rotatePercR > 0.5) rotatePercR = 0.5;

        translatePercL = 1.0 - rotatePercL;
        translatePercR = 1.0 - rotatePercR;
    }

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;
        if (firstMovement){
            if (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE){
                translatePercR = 0;
                translatePercL = 0;
                rotatePercR = 1;
                rotatePercL = 1;
                spinPower = 0;
                spinClicksL = 0;
                spinClicksR = 0;
            } else{
                firstMovement = false;
                translatePercR = 0.6;
                translatePercL = 0.6;
                rotatePercR = 0.4;
                rotatePercL = 0.4;
            }
        }
    }

    public void rightStick(){
        if (Math.abs(leftCurrentW) < constants.degreeTOLERANCE && Math.abs(rightCurrentW) < constants.degreeTOLERANCE && lx == 0 && ly == 0 && (rx != 0 || ry != 0)){
//            leftThrottle = leftThrottle;
            rightThrottle *= -1;

            spinClicksL = (int) (rx * 100 * leftThrottle);
            spinClicksR = (int) (rx * 100 * rightThrottle);
            spinPower = rx;

            rotatePercL = Math.abs(turnAmountL) / 60.0;
            if (rotatePercL > 0.5) rotatePercL = 0.5;

            rotatePercR = Math.abs(turnAmountR) / 60.0;
            if (rotatePercR > 0.5) rotatePercR = 0.5;

            translatePercL = 1.0 - rotatePercL;
            translatePercR = 1.0 - rotatePercR;

            turnAmountL = -leftCurrentW;
            leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
            leftRotatePower = snapLeftWheelPID.update(turnAmountL);

            turnAmountR = -rightCurrentW;
            rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
            rightRotatePower = snapRightWheelPID.update(turnAmountR);

            type = DriveType.TURN;
        } else{
            //spline
        }
    }

    public void stop(){
        target = 0;
        turnAmountL = 0;
        turnAmountR = 0;

        spinPower = 0;
        spinClicksR = 0;
        spinClicksL = 0;

        rightRotatePower = 0;
        leftRotatePower = 0;
        rightRotClicks = 0;
        leftRotClicks = 0;

        translatePercR = 0;
        translatePercL = 0;
        rotatePercR = 0;
        rotatePercL = 0;
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

    public double wheelOptimization(double target, double currentW, boolean right){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if (right) rightThrottle = 1;
        else leftThrottle = -1;

        if(Math.abs(turnAmount) > 90){
            double temp_target = clamp(target + 180);
            turnAmount = temp_target - currentW;

            if (right) this.rightThrottle *= -1;
            else this.leftThrottle *= -1;
        }
        return turnAmount;
    }

    public double wheelOptimizationR(double target, double currentW){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if(Math.abs(turnAmount) > 90){
            double temp_target = clamp(target + 180);
            target2 = (temp_target < 0 ? temp_target + 360 : temp_target);

            turnAmount1 = temp_target - currentW;
            turnAmount2 = target2 - current2;

            turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

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
//        motorPower[0] = (spinPower * translatePercL * leftThrottle + leftRotatePower * rotatePercL); //top left
//        motorPower[1] = (spinPower * translatePercL * leftThrottle + leftRotatePower * rotatePercL); //bottom left
//        motorPower[2] = (spinPower * translatePercR * rightThrottle+ rightRotatePower * rotatePercR); //top right
//        motorPower[3] = (spinPower * translatePercR * rightThrottle + rightRotatePower * rotatePercR); //bottom right
        motorPower[0] = (spinPower * leftThrottle); //top left
        motorPower[1] = (spinPower * leftThrottle); //bottom left
        motorPower[2] = (spinPower * rightThrottle); //top right
        motorPower[3] = (spinPower * rightThrottle); //bottom right


        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
            motorPower[i] *= constants.POWER_LIMITER;
            if (motorPower[i] > constants.POWER_LIMITER) motorPower[i] = constants.POWER_LIMITER;
            else if (motorPower[i] < -constants.POWER_LIMITER) motorPower[i] = -constants.POWER_LIMITER;
        }

        telLeftRotatePower = accelerator.update(leftRotatePower * rotatePercL * constants.POWER_LIMITER);
        telRightRotatePower = accelerator.update(rightRotatePower * rotatePercR * constants.POWER_LIMITER);

        telSpinPowerR = accelerator.update(spinPower * translatePercR * constants.POWER_LIMITER * rightThrottle);
        telSpinPowerL = accelerator.update(spinPower * translatePercL * constants.POWER_LIMITER * leftThrottle);

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