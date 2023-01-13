package org.firstinspires.ftc.teamcode.common.kinematics;

import com.vuforia.STORAGE_TYPE;

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

    public SwervePod PodL;
    public SwervePod PodR;

    double targetX;
    double targetY;
    private double finalAngle; //auto

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

    //input
    private double lx;
    private double ly;
    private double rx;
    private double ry;
    double rt;
    double lt;

    //output
    HashMap<String, Double> swerveOutputR = new HashMap<String, Double>();
    HashMap<String, Double> swerveOutputL = new HashMap<String, Double>();

    //current orientation
    GlobalPosSystem posSystem;

    public Accelerator accelerator;
    TrackJoystick joystickTracker;

    public boolean firstMovement = true;
    public boolean resestCycle = false;

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

    public RevisedKinematics(GlobalPosSystem posSystem, SwervePod podlL, SwervePod podR){
        this.posSystem = posSystem;

        accelerator = new Accelerator();
        joystickTracker = new TrackJoystick();

        this.PodL = podlL;
        this.PodR = podR;
    }

    public double target = 0;
    public boolean tryingToTurn = false;
    public void logic(double lx, double ly, double rx, double ry, double rt, double lt){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
        this.rt = rt;
        this.lt = lt;

        //telling the pods where it is
        PodL.setCurrents(posSystem.getLeftWheelW());
        PodR.setCurrents(posSystem.getRightWheelW());

        //tracking the joystick's movement
        joystickTracker.trackJoystickL(lx, ly);

        if (noMovementRequests()) type = DriveType.STOP;
        else type = DriveType.LINEAR;

        //determining targets, and how much we want to turn
        target = joystickTracker.getAngle(lx, ly);

        //determining rotational clicks
        PodL.setRotClicks(target);
        PodR.setRotClicks(target);

        posSystem.setOptimizedCurrentW(PodR.optimizedCurrentW, PodL.optimizedCurrentW);

//        boolean wheelsAreAlligned = posSystem.isAlligned();
        boolean eligibleForTurning = posSystem.eligibleForTurning();
        boolean shouldTurn = (lx == 0 && ly == 0) && (rx != 0) && eligibleForTurning; //possible problem: the robot will "jitter" if its turning and then becomes not eligible for turning (may have to increase tolerance?)
        boolean shouldSpline = (lx != 0 || ly != 0) && (rx != 0) && eligibleForTurning;

        //determining spin clicks and spin power
        double power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        type = PodL.setSpinClicksAndPower(power, rt, shouldTurn, shouldSpline, eligibleForTurning, rx);
        type = PodR.setSpinClicksAndPower(power, rt, shouldTurn, shouldSpline, eligibleForTurning, rx);
        PodL.setThrottleUsingPodLReference(PodR, shouldTurn, shouldSpline);

        //resetting modules when:
        // - the driver has not given controller input AND the wheels aren't alligned,
        // - the wheels aren't alligned with 0 degrees AND the driver is trying to turn.
        if (type == DriveType.STOP){
            if (!posSystem.isAlligned()){
                PodL.setRotClicks(0);
                PodR.setRotClicks(0);
                PodL.setSpinClicks(0);
                PodR.setSpinClicks(0);
                PodL.setPower(1);
                PodR.setPower(1);
            }
        }

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        swerveOutputR = PodR.getOutput();
        swerveOutputL = PodL.getOutput();
    }

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;

        if (!PodL.onlyRotate(firstMovement) && !PodR.onlyRotate(firstMovement)) firstMovement = false;

        if (firstMovement){
            PodL.setSpinClicks(0);
            PodR.setSpinClicks(0);
        }
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType){ //runs onc
        //target position
        this.targetX = x;
        this.targetY = y;
        this.finalAngle = finalAngle;

        //determining current position
        this.type = driveType;
        PodL.setCurrents(posSystem.getLeftWheelW());
        PodR.setCurrents(posSystem.getRightWheelW());

        PodL.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getMotorClicks()[0], false);
        PodR.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getMotorClicks()[2], true);
    }

    public void logicAuto(){ //should run everytime, but currently only runs once.
        PodL.setCurrents(posSystem.getLeftWheelW());
        PodR.setCurrents(posSystem.getRightWheelW());

        posSystem.setOptimizedCurrentW(PodR.optimizedCurrentW, PodL.optimizedCurrentW);

        //4) determining distance travel amount and power based on that
        PodL.autoLogic(posSystem.getLeftWheelW(),  posSystem.getMotorClicks()[0]);
        PodR.autoLogic(posSystem.getRightWheelW(), posSystem.getMotorClicks()[2]);

        swerveOutputR = PodR.getOutput();
        swerveOutputL = PodL.getOutput();
    }

    public void armLogicAuto(ArmType aType){
        boolean lowerArmCycle = false;
        boolean lowerAllTheWay = false;
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

    public double robotOptimization(double target, double currentR){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentR < 0 ? currentR + 360 : currentR);

        double turnAmount1 = target - currentR;
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

    public double[] getPower(){
        swerveOutputL = PodL.getOutput();
        swerveOutputR = PodR.getOutput();

//        double avgPower = (swerveOutputL.get("power") + swerveOutputL.get("power")) / 2.0;

        double[] motorPower = new double[4];
        motorPower[0] = (swerveOutputL.get("power") * swerveOutputL.get("throttle")); //top left
        motorPower[1] = (swerveOutputL.get("power") * swerveOutputL.get("throttle")); //bottom left
        motorPower[2] = (swerveOutputR.get("power") * swerveOutputR.get("throttle")); //top right
        motorPower[3] = (swerveOutputR.get("power") * swerveOutputR.get("throttle")); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i], true);
            motorPower[i] *= constants.POWER_LIMITER;
        }

        return motorPower;
    }

    public double[] getPowerAuto(){
        swerveOutputL = PodL.getOutput();
        swerveOutputR = PodR.getOutput();

        double[] motorPower = new double[4];
        double powerR = swerveOutputR.get("power");
        double powerL = swerveOutputL.get("power");
        motorPower[0] = (powerL * swerveOutputL.get("throttle")); //top left
        motorPower[1] = (powerL * swerveOutputL.get("throttle")); //bottom left
        motorPower[2] = (powerR * swerveOutputR.get("throttle")); //top right
        motorPower[3] = (powerR * swerveOutputR.get("throttle")); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i], true);
            motorPower[i] *= constants.POWER_LIMITER;
        }

        return motorPower;
    }

    public int[] getClicks(){
//        swerveOutputL = PodL.getOutput();
//        swerveOutputR = PodR.getOutput();

        double leftClicks = swerveOutputL.get("spinClicksTarget");
        double rightClicks = swerveOutputR.get("spinClicksTarget");
//        int avgClicks = (int)((leftClicks + rightClicks) / 2.0);

        int[] clicks = new int[4];
        clicks[0] = (int)(leftClicks  + swerveOutputL.get("rotClicksTarget")); //left
        clicks[1] = (int)(-leftClicks + swerveOutputL.get("rotClicksTarget")); //left
        clicks[2] = (int)(rightClicks + swerveOutputR.get("rotClicksTarget")); //right
        clicks[3] = (int)(-rightClicks + swerveOutputR.get("rotClicksTarget")); //right
        return clicks;
    }

    public int[] getClicksAuto(){
//        swerveOutputL = PodL.getOutput();
//        swerveOutputR = PodR.getOutput();

        int[] clicks = new int[4];
        clicks[0] = (int)(swerveOutputL.get("spinClicksTarget") + swerveOutputL.get("rotClicksTarget")); //left
        clicks[1] = (int)(-swerveOutputL.get("spinClicksTarget") + swerveOutputL.get("rotClicksTarget")); //left
        clicks[2] = (int)(swerveOutputR.get("spinClicksTarget") + swerveOutputR.get("rotClicksTarget")); //right
        clicks[3] = (int)(-swerveOutputR.get("spinClicksTarget") + swerveOutputR.get("rotClicksTarget")); //right
        return clicks;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }
}