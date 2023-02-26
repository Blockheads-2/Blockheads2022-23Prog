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
        NOT_INITIALIZED,
        RESET
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
        HOLD,
        RESET
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
        posSystem.setPoles(PodL.getPole(), PodR.getPole());
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

        posSystem.calculateWheelOrientation(PodL, PodR);
        boolean shouldTurn = (lx == 0 && ly == 0) && (rx != 0); //possible problem: the robot will "jitter" if its turning and then becomes not eligible for turning (may have to increase tolerance?)
        boolean shouldSpline = (lx != 0 || ly != 0) && (rx != 0);
        boolean eligibleForTurning = posSystem.eligibleForTurning(PodL.getPole(), PodR.getPole());
        boolean specialSpliningCondition = posSystem.specialSpliningCondition(PodL.getPole(), PodR.getPole(), PodL.getSlantedCycle());
        telemetry.addData("Special Splining Condition?", specialSpliningCondition);

        //determining spin clicks and spin power
        double power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));

        type = PodL.setSpinClicksAndPower(power, rt, shouldTurn, eligibleForTurning, shouldSpline, specialSpliningCondition, rx, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());
        type = PodR.setSpinClicksAndPower(power, rt, shouldTurn, eligibleForTurning, shouldSpline, specialSpliningCondition, rx, posSystem.getDistanceTravelledL(), posSystem.getDistanceTravelledR());

        //calculating the throttle difference between the wheels.  Uses these values to then keep the wheels running at the same speed later on.
        posSystem.calculateThrottleDifference(type == DriveType.LINEAR);

        //resetting modules when:
        // - the driver has not given controller input AND the wheels aren't alligned,
        // - the wheels aren't alligned with 0 degrees AND the driver is trying to turn.
        if (type == DriveType.STOP){
            posSystem.calculateThrottleDifference(true);
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

    public void resetAuto(){
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

//        posSystem.setOptimizedCurrentW(PodR.optimizedCurrentW, PodL.optimizedCurrentW);

        //4) determining distance travel amount and power based on that
        PodL.setResetValues();
        //for some reason, we negate the negative clicks for the left topL encoder
        PodR.setResetValues();
        this.type = DriveType.RESET;

        outputL = PodL.getOutputAuto();
        outputR = PodR.getOutputAuto();
    }

    public void setTurnPID(double kp, double ki, double kd){
        PodR.setPID(kp, ki, kd);
        PodL.setPID(kp, ki, kd);
    }

    public void turn(double finalAngle, double speed){
        this.finalAngle = finalAngle;

        posSystem.setPoles(PodL.getPole(), PodR.getPole());
        posSystem.calculatePos();
        PodL.setCurrents(posSystem.getLeftWheelW(), posSystem.getPositionArr()[4]);
        PodR.setCurrents(posSystem.getRightWheelW(), posSystem.getPositionArr()[4]);

        PodL.turn(finalAngle, speed);
        PodR.turn(finalAngle, speed);

        this.type = DriveType.RESET;

        outputL = PodL.getOutputAuto();
        outputR = PodR.getOutputAuto();
    }

    public void armLogicAuto(ArmType aType, double[] armClicks, double clawAngle, double claw){
        boolean lowerArmCycle = false;
        boolean lowerAllTheWay = false;
        double atTargetPos = 0;
        double ablTargetPos = 0;
        double abrTargetPos = 0;

        double atPower = 0;
        double ablPower = 0;
        double abrPower = 0;

        boolean usePID = false;

        this.armType = aType;

        if (clawAngle > 1) clawAngle = 1;
        else if (clawAngle < 0) clawAngle = 0;

        if (claw > 1) claw = 1;
        else if (claw < 0) claw = 0;

        switch (armType){
            case HIGH:
                atTargetPos = constants.topMotorHigh;
                ablTargetPos = constants.bottomMotorHigh;
                abrTargetPos = constants.bottomMotorHigh;

                usePID = false;

                break;

            case MID:
                atTargetPos = constants.topMotorMid;
                ablTargetPos = constants.bottomMotorMid;
                abrTargetPos = constants.bottomMotorMid;
                usePID = false;
                break;

            case LOW:
                lowerArmCycle = (armClicks[0] >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorLow + constants.degreeTOLERANCE);

                if (lowerArmCycle){
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

                lowerArmCycle = (armClicks[0] >= constants.topMotorLow + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorLow + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorLow + constants.degreeTOLERANCE);
                lowerAllTheWay = (armClicks[0] >= constants.topMotorBottom + constants.degreeTOLERANCE &&
                        armClicks[1] >= constants.bottomMotorBottom + constants.degreeTOLERANCE &&
                        armClicks[2] >= constants.bottomMotorBottom + constants.degreeTOLERANCE);

                if (lowerArmCycle){

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
                    atTargetPos = constants.topMotorBottom;
                    ablTargetPos = constants.bottomMotorBottom;
                    abrTargetPos = constants.bottomMotorBottom;
                }

                usePID = false;
                break;

            case HOLD:
                atTargetPos = armClicks[0];
                ablTargetPos = armClicks[1];
                abrTargetPos = armClicks[2];

                usePID = false;
                break;

            case RESET:
                atTargetPos = 0;
                ablTargetPos = 0;
                abrTargetPos = 0;
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
                    Math.abs(armClicks[4] - claw) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        } else if (armType == ArmType.GROUND){
            targetMet = (lowerArmCycle && lowerAllTheWay &&
                    Math.abs(armClicks[4] - claw) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        } else {
            targetMet = (Math.abs(armClicks[0] - atTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[1] - ablTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[2] - abrTargetPos) < constants.clickTOLERANCE &&
                    Math.abs(armClicks[4] - claw) <= 0.04 &&
                    Math.abs(armClicks[3] - clawAngle) <= 0.04);
        }

        armOutput[0] = atPower;
        armOutput[1] = ablPower;
        armOutput[2] = abrPower;
        armOutput[3] = atTargetPos;
        armOutput[4] = ablTargetPos;
        armOutput[5] = abrTargetPos;
        armOutput[6] = claw;
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
        telemetry.addData("Header Correction Throttle top motors left:",  PodL.headerCorrectionTop(posSystem.getAvgDiffBetweenMotors()[0]));
        telemetry.addData("Header Correction Throttle bottom motors left:",  PodL.headerCorrectionBottom(posSystem.getAvgDiffBetweenMotors()[1]));
        telemetry.addData("Header Correction Throttle top motors right:",  PodR.headerCorrectionTop(posSystem.getAvgDiffBetweenMotors()[0]));
        telemetry.addData("Header Correction Throttle bottom motors right:",  PodR.headerCorrectionBottom(posSystem.getAvgDiffBetweenMotors()[1]));

        clicks[0] = (int)((outputL[0] * PodL.headerCorrectionTop(posSystem.getAvgDiffBetweenMotors()[0])) - outputL[1]); //left
        clicks[1] = (int)(-(outputL[0] * PodL.headerCorrectionBottom(posSystem.getAvgDiffBetweenMotors()[1])) - outputL[1]); //left
        clicks[2] = (int)((outputR[0] * PodR.headerCorrectionTop(posSystem.getAvgDiffBetweenMotors()[0])) + outputR[1]); //right
        clicks[3] = (int)(-(outputR[0] * PodR.headerCorrectionBottom(posSystem.getAvgDiffBetweenMotors()[1])) + outputR[1]); //right
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