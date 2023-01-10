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

        //determining targets, and how much we want to turn
        double target = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) target = 0;
        else if (lx==0 && ly < 0) target=180;

        //determining rotational clicks
        PodL.setRotClicks(target);
        PodR.setRotClicks(target);

        boolean shouldTurn = (lx == 0 && ly == 0) && (rx != 0 || ry != 0) && posSystem.eligibleForTurning(PodR.getSpinDirection(), PodL.getSpinDirection());
        boolean shouldSpline = !shouldTurn && (lx != 0 || ly != 0) && (rx != 0 && ry == 0);

        //determining power
        double power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));

        //determining spin clicks and spin power
        type = PodL.setSpinClicks(power, rt, shouldTurn, shouldSpline, rx);
        type = PodR.setSpinClicks(power, rt, shouldTurn, shouldSpline, rx);
        PodL.setThrottleUsingPodLReference(PodR, shouldTurn, shouldSpline);

        //telling the pods where we are (note that the RevisedKinematics class knows it's current position before the Pods know about it because of the new thread running in AutoHub / FinalBaseDrive).
        PodL.setCurrents(posSystem.getLeftWheelW(), (shouldTurn || shouldSpline));
        PodR.setCurrents(posSystem.getRightWheelW(), (shouldTurn || shouldSpline));

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();
    }

    public void setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType){ //runs onc
        //target position
        this.targetX = x;
        this.targetY = y;
        this.finalAngle = finalAngle;

        //determining current position
        this.type = driveType;
        PodL.setCurrents(posSystem.getLeftWheelW(), (driveType == DriveType.TURN || driveType == DriveType.VARIABLE_SPLINE));
        PodR.setCurrents(posSystem.getRightWheelW(), (driveType == DriveType.TURN || driveType == DriveType.VARIABLE_SPLINE));

        PodL.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getMotorClicks()[0], false);
        PodR.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getMotorClicks()[2], true);
    }

    public void logicAuto(){ //should run everytime, but currently only runs once.
        PodL.setCurrents(posSystem.getLeftWheelW(), (type == DriveType.TURN || type == DriveType.VARIABLE_SPLINE));
        PodR.setCurrents(posSystem.getRightWheelW(), (type == DriveType.TURN || type == DriveType.VARIABLE_SPLINE));

        //4) determining distance travel amount
        PodL.autoLogic(posSystem.getLeftWheelW(), (PodL.getDriveType() == DriveType.TURN || PodL.getDriveType() == DriveType.VARIABLE_SPLINE),  posSystem.getMotorClicks()[0]);
        PodR.autoLogic(posSystem.getRightWheelW(), (PodR.getDriveType() == DriveType.TURN || PodR.getDriveType() == DriveType.VARIABLE_SPLINE), posSystem.getMotorClicks()[2]);

        //determining spin power
        PodL.setPowerUsingLeft(PodR); //determines for both PodL and PodR.

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

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;

        if (!(PodL.onlyRotate(firstMovement) && PodR.onlyRotate(firstMovement))) firstMovement = false;

        if (firstMovement){
            PodL.setSpinClicks(0);
            PodR.setSpinClicks(0);
        }
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
        PodL.getOutput();
        PodR.getOutput();

        double[] motorPower = new double[4];
        motorPower[0] = (PodL.output.get("power") * PodL.output.get("throttle") * PodL.output.get("direction")); //top left
        motorPower[1] = (PodL.output.get("power") * PodL.output.get("throttle") * PodL.output.get("direction")); //bottom left
        motorPower[2] = (PodR.output.get("power") * PodR.output.get("throttle") * PodR.output.get("direction")); //top right
        motorPower[3] = (PodR.output.get("power") * PodR.output.get("throttle") * PodR.output.get("direction")); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i], true);
            motorPower[i] *= constants.POWER_LIMITER;
        }

        return motorPower;
    }

    public int[] getClicks(){
        PodL.getOutput();
        PodR.getOutput();

//        int spinTargets = (int)((PodL.output.get("spinClicksTarget") + PodR.output.get("spinClicksTarget")) / 2);

        int[] clicks = new int[4];
        clicks[0] = (int)(PodL.output.get("spinClicksTarget") * PodL.output.get("direction") + PodL.output.get("rotClicksTarget")); //left
        clicks[1] = (int)(-PodL.output.get("spinClicksTarget") * PodL.output.get("direction") + PodL.output.get("rotClicksTarget")); //left
        clicks[2] = (int)(PodR.output.get("spinClicksTarget") * PodR.output.get("direction") + PodR.output.get("rotClicksTarget")); //right
        clicks[3] = (int)(-PodR.output.get("spinClicksTarget") * PodR.output.get("direction") + PodR.output.get("rotClicksTarget")); //right
        return clicks;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }
}