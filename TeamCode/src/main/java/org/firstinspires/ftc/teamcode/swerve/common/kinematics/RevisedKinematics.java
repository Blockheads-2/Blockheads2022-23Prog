package org.firstinspires.ftc.teamcode.swerve.common.kinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.pid.ArmPID;
import org.firstinspires.ftc.teamcode.swerve.common.pid.HeaderControlPID;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

public class RevisedKinematics {
    protected Constants constants = new Constants();

    SwervePod PodL;
    SwervePod PodR;

    Telemetry telemetry;

    double targetX;
    double targetY;
    public double finalAngle; //auto

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
    double[] outputL = new double[3];
    double[] outputR = new double[3];

    //current orientation
    GlobalPosSystem posSystem;

    TrackJoystick joystickTracker;

    public boolean firstMovement = true;
//    public boolean resestCycle = false;

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

    public ArmPID atPID = new ArmPID();
    public ArmPID abrPID = new ArmPID();
    public ArmPID ablPID = new ArmPID();
//    HashMap<String, Double> armOutput = new HashMap<String, Double>();
    double[] armOutput = new double[8];

    public RevisedKinematics(GlobalPosSystem posSystem, SwervePod podlL, SwervePod podR){
        this.posSystem = posSystem;

        joystickTracker = new TrackJoystick();

        this.PodL = podlL;
        this.PodR = podR;
    }

    public void grabTelemetry(Telemetry t){
        telemetry = t;
        PodL.grabTelemetry(t);
        PodR.grabTelemetry(t);
    }

    public double target = 0;
    public void logic(double lx, double ly, double rx, double ry, double rt, double lt){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
        this.rt = rt;
        this.lt = lt;

        //telling the pods where it is
        posSystem.calculatePos();
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

        //tracking the joystick's movement
        joystickTracker.trackJoystickL(lx, ly);

        //determining targets, and how much we want to turn
        target = joystickTracker.getAngle(lx, ly);

        if (lx != 0 || ly != 0){
            PodL.setRotClicks(target);
            PodR.setRotClicks(target);
        } else {
            PodL.forceSetRotClicks(0); //have to change anything with InitPole?
            PodR.forceSetRotClicks(0);
        }

        boolean shouldTurn = (lx == 0 && ly == 0) && (rx != 0); //possible problem: the robot will "jitter" if its turning and then becomes not eligible for turning (may have to increase tolerance?)
        boolean shouldSpline = (lx != 0 || ly != 0) && (rx != 0);
        boolean eligibleForTurning = posSystem.eligibleForTurning(PodL.getPole(), PodR.getPole());
        boolean specialSpliningCondition = posSystem.specialSpliningCondition(PodL.getPole(), PodR.getPole());

        //determining spin clicks and spin power
        double power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));

        type = PodL.setSpinClicksAndPower(power, rt, shouldTurn, eligibleForTurning, shouldSpline, specialSpliningCondition, rx, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());
        type = PodR.setSpinClicksAndPower(power, rt, shouldTurn, eligibleForTurning, shouldSpline, specialSpliningCondition, rx, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());

//        PodL.setThrottleUsingPodLReference(PodR, shouldTurn, shouldSpline);

        //resetting modules when:
        // - the driver has not given controller input AND the wheels aren't alligned,
        // - the wheels aren't alligned with 0 degrees AND the driver is trying to turn.
        if (type == DriveType.STOP){
            if (!posSystem.isAlligned(PodL.getPole(), PodR.getPole())){
                PodL.setRotClicks(0);
                PodR.setRotClicks(0);
                PodL.setSpinClicks(0);
                PodR.setSpinClicks(0);
                PodL.setPower(1);
                PodR.setPower(1);
            }
        }

        //determining "firstMovement" actions, if it is the ?//'/robot's "firstMovement."
        firstMovement();

        outputL = PodL.getOutput();
        outputR = PodR.getOutput();
    }

    public void firstMovement(){
        if (joystickTracker.getAbsoluteChange() > 90 || noMovementRequests()) firstMovement = true;
        telemetry.addData("Joystick change > 90 ? ", joystickTracker.getAbsoluteChange());
        telemetry.addData("No movement requests?", noMovementRequests());


        if (!PodL.onlyRotate(firstMovement) && !PodR.onlyRotate(firstMovement)) firstMovement = false;

        if (firstMovement){
            PodL.setSpinClicks(0);
            PodR.setSpinClicks(0);
        }
    }

    public double setPosAuto(double x, double y, double finalAngle, double speed, DriveType driveType){ //runs onc
        //target position
        this.targetX = x;
        this.targetY = y;
        this.finalAngle = finalAngle;
        //determining current position
        this.type = driveType;
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

        double timeOutL = PodL.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR(), posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        double timeOutR = PodR.setPosAuto(x, y, finalAngle, speed, driveType, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR(), posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);
        return Math.max(timeOutL, timeOutR);
    }

    public void logicAuto(){ //should run everytime, but currently only runs once.
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

//        posSystem.setOptimizedCurrentW(PodR.optimizedCurrentW, PodL.optimizedCurrentW);

        //4) determining distance travel amount and power based on that
        PodL.autoLogic(posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());
        //for some reason, we negate the negative clicks for the left topL encoder
        PodR.autoLogic(posSystem.getDistanceTravelledR(), posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());

        outputL = PodL.getOutputAuto();
        outputR = PodR.getOutputAuto();
    }

    public void setTurnPID(double kp, double ki, double kd){
        PodR.setPID(kp, ki, kd);
        PodL.setPID(kp, ki, kd);
    }

    public void turn(double finalAngle, double speed){
        this.finalAngle = finalAngle;

        posSystem.calculatePos();
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

        PodL.turn(finalAngle, speed);
        PodR.turn(finalAngle, speed);

        outputL = PodL.getOutputAuto();
        outputR = PodR.getOutputAuto();
    }

    public void armLogicAuto(ArmType aType, double[] armClicks){
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
                clawAngle = constants.armServoHigh;
                clawClamp = armClicks[4];

                usePID = false;

                break;

            case MID:
                atTargetPos = constants.topMotorMid;
                ablTargetPos = constants.bottomMotorMid;
                abrTargetPos = constants.bottomMotorMid;
                clawAngle = constants.armServoMid;
                clawClamp = armClicks[4];
                usePID = false;
                break;

            case LOW:
                clawClamp = armClicks[4];

                lowerArmCycle = (armClicks[0] >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorLow + constants.degreeTOLERANCE);

                if (lowerArmCycle){
                    clawAngle = constants.armServoLow;

                    atTargetPos = constants.topMotorLow;

                    int initPos = 0;
                    if (armClicks[1] >= initPos && armClicks[2] >= initPos) {
                        ablTargetPos = initPos;
                        abrTargetPos = initPos;
                    } else{
                        ablTargetPos = constants.bottomMotorLow;
                        abrTargetPos = constants.bottomMotorLow;
                    }
                }
                usePID = false;
                break;

            case GROUND:
                clawClamp = armClicks[4];

                lowerArmCycle = (armClicks[0] >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorLow + constants.degreeTOLERANCE);
                lowerAllTheWay = (armClicks[0] >= constants.topMotorBottom + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorBottom + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorBottom + constants.degreeTOLERANCE);

                if (lowerArmCycle){
                    clawAngle = constants.armServoLow;

                    atTargetPos = constants.topMotorLow;

                    int initPos = 0;
                    if (armClicks[1] >= initPos && armClicks[2] >= initPos) {
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

                usePID = false;
                break;

            case GRAB:
                clawClamp = constants.closeClaw;
                clawAngle = armClicks[3];

                usePID = false;
                break;

            case DROP:
                clawClamp = constants.openClaw;
                clawAngle = armClicks[3];

                usePID = false;
                break;

            case HOLD:
                atTargetPos = armClicks[0];
                ablTargetPos = armClicks[1];
                abrTargetPos = armClicks[2];
                clawClamp = armClicks[4];
                clawAngle = armClicks[3];

                usePID = false;
                break;
        }

        if (usePID) {
            atPID.setTargets(atTargetPos, constants.kpArm, constants.kiArm, constants.kdArm);
            ablPID.setTargets(ablTargetPos, constants.kpArm, constants.kiArm, constants.kdArm);
            abrPID.setTargets(abrTargetPos, constants.kpArm, constants.kiArm, constants.kdArm);
            atPower = atPID.update(armClicks[0]);
            ablPower = ablPID.update(armClicks[1]);
            abrPower = abrPID.update(armClicks[2]);
            //if you are going to use a PID, you need feed forward
        } else {
            atPower = 0.4;
            ablPower = 0.8;
            abrPower = 0.8;
        }

        if (armType == ArmType.LOW){
            targetMet = (lowerArmCycle &&
                    Math.abs(armClicks[4] - clawClamp) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        } else if (armType == ArmType.GROUND){
            targetMet = (lowerArmCycle && lowerAllTheWay &&
                    Math.abs(armClicks[4] - clawClamp) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        } else {
            targetMet = (Math.abs(armClicks[0] - atTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[1] - ablTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[2] - abrTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[4] - clawClamp) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        }

        armOutput[0] = atPower;
        armOutput[1] = ablPower;
        armOutput[2] = abrPower;
        armOutput[3] = atTargetPos;
        armOutput[4] = ablTargetPos;
        armOutput[5] = abrTargetPos;
        armOutput[6] = clawClamp;
        armOutput[7] = clawAngle;
    }

    public double[] getArmOutput(){
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
//        double avgPower = (swerveOutputL.get("power") + swerveOutputL.get("power")) / 2.0;

        double[] motorPower = new double[4];
        motorPower[0] = (outputL[2]); //top left
        motorPower[1] = (outputL[2]); //bottom left
        motorPower[2] = (outputR[2]); //top right
        motorPower[3] = (outputR[2]); //bottom right

        return motorPower;
    }

//    public double[] getPowerAuto(){
//    }

    public int[] getClicks(){
//        double leftClicks = swerveOutputL.get("spinClicksTarget");
//        double rightClicks = swerveOutputR.get("spinClicksTarget");
//        int avgClicks = (int)((leftClicks + rightClicks) / 2.0);

        int[] clicks = new int[4];
        clicks[0] = (int)(outputL[0] - outputL[1]); //left
        clicks[1] = (int)(-outputL[0] - outputL[1]); //left
        clicks[2] = (int)(outputR[0] + outputR[1]); //right
        clicks[3] = (int)(-outputR[0] + outputR[1]); //right
        return clicks;
    }

//    public int[] getClicksAuto(){
//    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }
}