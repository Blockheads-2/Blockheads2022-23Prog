package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.ArmPID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;
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
    SpinPID pidR;
    SpinPID pidL;

    public boolean firstMovement = true;

    //arm stuff!
    public enum ArmType{
        HIGH,
        MID,
        LOW,
        GROUND,
        GRAB,
        DROP,
        HOLD
    }

    ArmType armType = ArmType.HOLD;
    boolean targetMet = false;

    ArmPID atPID = new ArmPID();
    ArmPID abrPID = new ArmPID();
    ArmPID ablPID = new ArmPID();
    HashMap<String, Double> armOutput = new HashMap<String, Double>();

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();
        pidR = new SpinPID();
        pidL = new SpinPID();

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
            power = 0;
            return;
        }

        //determining current position
        setCurrentPos();

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
        spinClicksL = (int)(power * 100 * Math.signum(leftThrottle)); //increase the 100 to increase power (but note that increasing this will decrease the rotation power slightly)
        spinClicksR = (int)(power * 100 * Math.signum(rightThrottle));

        //determining rotational power
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        //determining values from right stick input.
        rightStick();
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType){ //runs once
        posSystem.resetXY();
        this.type = driveType;

        //target position
        this.targetX = x;
        this.targetY = y;
        this.finalAngle = finalAngle;
        this.speed = speed;

        //determining current position
        setCurrentPos();

        linearMath.setInits(this.currentX, this.currentY);
        linearMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd);

        splineMath.setInits(posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2]);
        splineMath.setPos(x, y, finalAngle, constants.kp, constants.ki, constants.kd);

        firstMovement = (type == DriveType.LINEAR);
    }

    public void logicAuto(){ //runs once
        setCurrentPos();

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

//            case TURN:
//                distanceL = turnMath.getDistanceRemaining(posSystem.getMotorClicks()[2]);
//                distanceR = -distanceL;
//                turnAmountL = 0;
//                turnAmountR = 0;
//                break;

            case SNAP:
                distanceL = 0;
                distanceR = 0;
                turnAmountL = finalAngle;
                turnAmountR = finalAngle;
                break;

            case STOP:
                distanceL = 0;
                distanceR = 0;
                turnAmountL = 0;
                turnAmountR = 0;
                break;
        }

        rightThrottle = constants.initDirectionRight;
        leftThrottle = constants.initDirectionLeft;

        if (type == DriveType.SNAP) {
            pidR.setTargets(turnAmountR, constants.kp, constants.ki, constants.kd);
            pidL.setTargets(turnAmountL, constants.kp, constants.ki, constants.kd);
        } else{
            pidR.setTargets(distanceR, constants.kp, constants.ki, constants.kd);
            pidL.setTargets(distanceL, constants.kp, constants.ki, constants.kd);
        }


        //determining spin power
        power = (pidR.update(posSystem.getMotorClicks()[2]) + pidL.update(posSystem.getMotorClicks()[0]) / 2.0) * speed;
        spinClicksL = (int)(distanceL * constants.CLICKS_PER_INCH * Math.signum(leftThrottle));
        spinClicksR = (int)(distanceR * constants.CLICKS_PER_INCH * Math.signum(rightThrottle));

        //determining rotational power
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
    }

    public boolean lowerArmCycle = false;
    public boolean lowerAllTheWay = false;
    public void armLogicAuto(ArmType aType){
        double atTargetPos = 0;
        double ablTargetPos = 0;
        double abrTargetPos = 0;
        double clawAngle = 0;
        double clawClamp = 0;

        double atPower = 0;
        double ablPower = 0;
        double abrPower = 0;

        boolean usePID = false;

        this.armType = aType;

        switch (armType){
            case HIGH:
                atTargetPos = constants.topMotorHigh;
                ablTargetPos = constants.bottomMotorHigh;
                abrTargetPos = constants.bottomMotorHigh;
                usePID = true;
                clawAngle = constants.armServoHigh;

                break;

            case MID:
                atTargetPos = constants.topMotorMid;
                ablTargetPos = constants.bottomMotorMid;
                abrTargetPos = constants.bottomMotorMid;
                clawAngle = constants.armServoMid;
                usePID = true;
                break;

            case LOW:
                lowerArmCycle = (posSystem.getArmClicks().get("at") >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abl") >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abr") >= constants.bottomMotorLow + constants.degreeTOLERANCE);

                if (lowerArmCycle){
                    clawAngle = constants.armServoLow;

                    atTargetPos = constants.topMotorLow;

                    int initPos = 0;
                    if (posSystem.getArmClicks().get("abl") >= initPos && posSystem.getArmClicks().get("abr") >= initPos) {
                        ablTargetPos = initPos;
                        abrTargetPos = initPos;
                    } else{
                        ablTargetPos = constants.bottomMotorLow;
                        abrTargetPos = constants.bottomMotorLow;
                    }
                }
                usePID = true;
                break;

            case GROUND:
                lowerArmCycle = (posSystem.getArmClicks().get("at") >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abl") >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abr") >= constants.bottomMotorLow + constants.degreeTOLERANCE);
                lowerAllTheWay = (posSystem.getArmClicks().get("at") >= constants.topMotorBottom + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abl") >= constants.bottomMotorBottom + constants.degreeTOLERANCE &&
                        posSystem.getArmClicks().get("abr") >= constants.bottomMotorBottom + constants.degreeTOLERANCE);

                if (lowerArmCycle){
                    clawAngle = constants.armServoLow;

                    atTargetPos = constants.topMotorLow;

                    int initPos = 0;
                    if (posSystem.getArmClicks().get("abl") >= initPos && posSystem.getArmClicks().get("abr") >= initPos) {
                        ablTargetPos = initPos;
                        abrTargetPos = initPos;
                    } else{
                        ablTargetPos = constants.bottomMotorLow;
                        abrTargetPos = constants.bottomMotorLow;
                    }
                }

                if (!lowerArmCycle && lowerAllTheWay){
                    clawAngle = constants.armServoBottom;
                    atTargetPos = constants.topMotorBottom;
                    ablTargetPos = constants.bottomMotorBottom;
                    abrTargetPos = constants.bottomMotorBottom;
                }

                usePID = true;
                break;

            case GRAB:
                clawClamp = constants.closeClaw;

                usePID = false;
                break;

            case DROP:
                clawClamp = constants.openClaw;

                usePID = false;
                break;

            case HOLD:
                atTargetPos = posSystem.getArmClicks().get("at");
                ablTargetPos = posSystem.getArmClicks().get("abl");
                abrTargetPos = posSystem.getArmClicks().get("abr");

                usePID = false;
                break;
        }

        if (usePID) {
            atPID.setTargets(atTargetPos, constants.kp, constants.ki, constants.kd);
            ablPID.setTargets(ablTargetPos, constants.kp, constants.ki, constants.kd);
            abrPID.setTargets(abrTargetPos, constants.kp, constants.ki, constants.kd);
            atPower = atPID.update(posSystem.getArmClicks().get("at")) * 0.8;
            ablPower = ablPID.update(posSystem.getArmClicks().get("abl")) * 0.8;
            abrPower = abrPID.update(posSystem.getArmClicks().get("abr")) * 0.8;
            //if you are going to use a PID, you need feed forward
        } else {
            atPower = 0.4;
            ablPower = 0.8;
            abrPower = 0.8;
        }

        if (armType == ArmType.LOW){
            targetMet = (lowerArmCycle &&
                    Math.abs(posSystem.getArmClicks().get("claw") - clawClamp) <= 0.04 &&
                    Math.abs(posSystem.getArmClicks().get("armServo") - clawAngle) <= 0.04);
        } else if (armType == ArmType.GROUND){
            targetMet = (lowerArmCycle && lowerAllTheWay &&
                    Math.abs(posSystem.getArmClicks().get("claw") - clawClamp) <= 0.04 &&
                    Math.abs(posSystem.getArmClicks().get("armServo") - clawAngle) <= 0.04);
        } else {
            targetMet = (Math.abs(posSystem.getArmClicks().get("at") - atTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(posSystem.getArmClicks().get("abl") - ablTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(posSystem.getArmClicks().get("abr") - abrTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(posSystem.getArmClicks().get("claw") - clawClamp) <= 0.04 &&
                    Math.abs(posSystem.getArmClicks().get("armServo") - clawAngle) <= 0.04);
        }

        armOutput.put("atPower", atPower);
        armOutput.put("ablPower", ablPower);
        armOutput.put("abrPower", abrPower);
        armOutput.put("atTargetPos", atTargetPos);
        armOutput.put("ablTargetPos", ablTargetPos);
        armOutput.put("abrTargetPos", abrTargetPos);
        armOutput.put("clawClampPos", clawClamp);
        armOutput.put("clawAnglePos", clawAngle); //armServo
    }

    public HashMap<String, Double> getArmOutput(){
        return armOutput;
    }

    public boolean isArmTargetMet(){
        return targetMet;
    }

    public ArmType getArmType(){
        return armType;
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

    public double robotOptimization(double target, double currentR){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentR < 0 ? currentR + 360 : currentR);

        double turnAmount1 = target - currentR;
        double turnAmount2 = target2 - current2;

        return (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
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

    void setCurrentPos(){
        currentX = posSystem.getPositionArr()[0];
        currentY = posSystem.getPositionArr()[1];
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];
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

    public int[] getMotorClicks(){
        return posSystem.getMotorClicks();
    }

    public void switchLeftSpinDirection(){
        leftThrottle *= -1;
    }

    public void switchRightSpinDirection(){
        rightThrottle *= -1;
    }
}