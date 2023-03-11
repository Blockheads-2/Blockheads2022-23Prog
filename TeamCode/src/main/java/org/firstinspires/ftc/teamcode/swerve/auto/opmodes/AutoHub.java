package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.auto.cv.CameraMaster;
import org.firstinspires.ftc.teamcode.swerve.common.Reset;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;
import org.firstinspires.ftc.teamcode.swerve.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

@Config
public class AutoHub implements Runnable{
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;
    Reset reset;

    SwervePod podR;
    SwervePod podL;

    public static double powerRotate = 1;
    public static double powerTranslate = 0.6;
    public static double powerTurn = 1;
    public static double distance = 30;
    public static double distance2 = 0;
    public static double finalTurnAngle = 90;
    public static double finalSnapAngle = 0;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    public static double internalKPL = 10;
    public static double internalKPR = 10;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double topSegLength = 406; //406
    double botSegLength = 420; //420
    double servoUnits = 0;

    ElapsedTime loopTime = new ElapsedTime();
    double prevMS = 0;
    double deltaMS = 0;

    ElapsedTime stopConditionTimer = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();

    TurnMath turnMath = new TurnMath();
    SnapSwerveModulePID turnPID = new SnapSwerveModulePID(); //although it says Snap, it works for turning as well.

    double[] motorPower = new double[4];
    int[] targetClicks = new int[4];
    boolean targetNotMet = true;
    double[] armOutput = new double[8];

    View relativeLayout;

    TelemetryPacket packet;


    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);


        podR = new SwervePod(constants.initDirectionRight, SwervePod.Side.RIGHT);
        podL = new SwervePod(constants.initDirectionLeft, SwervePod.Side.LEFT);

        kinematics = new RevisedKinematics(posSystem, podL, podR);
        posSystem.grabKinematics(kinematics);
        kinematics.grabTelemetry(linearOpMode.telemetry);
        reset = new Reset(robot, posSystem);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        packet = new TelemetryPacket();
        podL.grabDashboard(packet);
        podR.grabDashboard(packet);
        dashboard.setTelemetryTransmissionInterval(25);

//        packet.put("PowerR", powerR);
        packet.put("PowerL", powerL);
//        packet.put("Error R", podR.getPID().getError());
        packet.put("Error L", podL.getPID().getError());
//        packet.put("Distance Travelled R", posSystem.getDistanceTravelledR());
        packet.put("Distance Travelled L", posSystem.getDistanceTravelledL());
        packet.put("Left W",  posSystem.getLeftWheelW());
        packet.put("Robot Header", posSystem.getPositionArr()[4]);

//        packet.put("Error at", kinematics.atPID.getError());
//        packet.put("Target at", kinematics.atPID.getTarget());

//        packet.put("Error abl", kinematics.ablPID.getError());
//        packet.put("Target abl", kinematics.ablPID.getTarget());

//        packet.put("Error abr", kinematics.abrPID.getError());
//        packet.put("Target abr", kinematics.abrPID.getTarget());
        packet.put("TimeOut", timeoutS);


        packet.put("delta time", deltaMS);

        dashboard.sendTelemetryPacket(packet);

        loopTime.reset();

        robot.setWheelRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setInternalPIDFCoef(internalKPR, 0, 0, 0, internalKPL, 0, 0, 0);

        robot.setWheelRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToInit(){
        robot.claw.setPosition(constants.openClaw);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        robot.abl.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.abr.setTargetPosition(constants.INIT_ARMBASE_POS);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(0.6);
        robot.abr.setPower(0.6);

        robot.armServo.setPosition(0);
    }

    public void resetArmEncoderPos(){
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double powerR = 0;
    double powerL = 0;
    ElapsedTime runTime = new ElapsedTime();
    double timeoutS = 0;
    public void Move(RevisedKinematics.DriveType movementType, double x, double y, double finalAngle, double speed, RevisedKinematics.ArmType armMovementType, double clawAngle, double claw){
        robot.setInternalPIDFCoef(internalKPR, 0, 0, 0, internalKPL, 0, 0, 0);

//        if (movementType == RevisedKinematics.DriveType.LINEAR) robot.setInternalPIDFCoef(3, 0, 0, 0, 5, 0, 0, 0);

        UpdateTelemetry();

        //1) Calculate our current position
        posSystem.resetXY(); // <-- Must have!
        posSystem.setPoles(podL.getPole(), podR.getPole());
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        timeoutS = kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        kinematics.armLogicAuto(armMovementType, getArmClicks(), clawAngle, claw); //determine targets/power for the arm
        kinematics.logicAuto();

        targetNotMet = true;
        targetClicks = kinematics.getClicks();

        armOutput = kinematics.getArmOutput();

        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

        robot.topL.setTargetPosition(targetTopL);
        robot.botL.setTargetPosition(targetBotL);
        robot.topR.setTargetPosition(targetTopR);
        robot.botR.setTargetPosition(targetBotR);

        robot.at.setTargetPosition((int)armOutput[3]);
        robot.abl.setTargetPosition((int)armOutput[4]);
        robot.abr.setTargetPosition((int)armOutput[5]);
        robot.armServo.setPosition(armOutput[7]);
        robot.claw.setPosition(armOutput[6]);


        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean armTargetMet = kinematics.isArmTargetMet();
        //3) Tell the robot to travel that distance we just determined.

        runTime.reset();

        double extraTime = (movementType == RevisedKinematics.DriveType.SNAP ? 1.5 : 0.7);
        while (linearOpMode.opModeIsActive() && (targetNotMet || !armTargetMet) && runTime.seconds() < timeoutS + constants.autoStopConditionTime + extraTime){ //have a time based something in case our target is never met.
            posSystem.setPoles(podL.getPole(), podR.getPole());
            posSystem.calculatePos();
            kinematics.armLogicAuto(armMovementType, getArmClicks(), clawAngle, claw); //determine targets/power for the arm
            // see how long program takes without calling armLogicAuto

            kinematics.logicAuto();

            targetClicks = kinematics.getClicks();
            motorPower = kinematics.getPower();

            powerL = motorPower[0];
            powerR = motorPower[2];

            targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
            targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
            targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
            targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
            robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
            robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
            robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);
//            robot.topL.setVelocity(speed * constants.MAX_VELOCITY_DT);
//            robot.botL.setVelocity(speed * constants.MAX_VELOCITY_DT);
//            robot.topR.setVelocity(speed * constants.MAX_VELOCITY_DT);
//            robot.botR.setVelocity(speed * constants.MAX_VELOCITY_DT);

            robot.at.setPower(armOutput[0]);
            robot.abl.setPower(armOutput[1]);
            robot.abr.setPower(armOutput[2]);

//            robot.claw.setPosition(armOutput[6]);
//            robot.armServo.setPosition(armOutput[7]);

            //perhaps set a timer for this one like turning
            targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > constants.degreeTOLERANCE ||
                    Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.degreeTOLERANCE ||
                    Math.abs(robot.topR.getCurrentPosition() - targetTopR) > constants.degreeTOLERANCE ||
                    Math.abs(robot.botR.getCurrentPosition() - targetBotR) > constants.degreeTOLERANCE);
            armTargetMet = kinematics.isArmTargetMet();

            if (!targetNotMet){
                if (stopConditionTimer.seconds() > constants.autoStopConditionTime) targetNotMet = false;
            } else {
                stopConditionTimer.reset();
                targetNotMet = true;
            }

            UpdateTelemetry();

            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();
        }

        reset();
    }

    public void reset(){
        podL.getAccelerator().resetAccelerationAuto();
        podR.getAccelerator().resetAccelerationAuto();

        stopConditionTimer.reset();
        targetNotMet = true;

        int targetTopL = 0;
        int targetBotL = 0;
        int targetTopR = 0;
        int targetBotR = 0;


        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
            while (linearOpMode.opModeIsActive() && (targetNotMet)){
                posSystem.setPoles(podL.getPole(), podR.getPole());
                posSystem.calculatePos();
                kinematics.resetAuto();
                targetClicks = kinematics.getClicks();
                motorPower = kinematics.getPower();

                targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
                targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
                targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
                targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

                robot.topL.setTargetPosition(targetTopL);
                robot.botL.setTargetPosition(targetBotL);
                robot.topR.setTargetPosition(targetTopR);
                robot.botR.setTargetPosition(targetBotR);

                robot.setWheelRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
                robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
                robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
                robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);

                targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > 2 ||
                        Math.abs(robot.botL.getCurrentPosition() - targetBotL) > 2 ||
                        Math.abs(robot.topR.getCurrentPosition() - targetTopR) > 2 ||
                        Math.abs(robot.botR.getCurrentPosition() - targetBotR) > 2);

                if (!targetNotMet){
                    if (stopConditionTimer.seconds() > constants.autoStopConditionTime) targetNotMet = false;
                } else {
                    stopConditionTimer.reset();
                    targetNotMet = true;
                }
            }
        }

        robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fullReset(){
        reset.resetAuto(false);
        while (!reset.finishedReset()){
            reset.resetAuto(true);
        }
        reset.resetAuto(false);
        posSystem.resetHeader();
        posSystem.resetXY();
        posSystem.hardResetGPS();
    }

    public void Turn(double finalAngle, double speed){
        robot.setInternalPIDFCoef(1.3, 0, 0, 0, 1.3, 0, 0, 0);

        timer.reset();

        //1) Calculate our current position
        posSystem.resetXY(); // <-- Must have!
        posSystem.setPoles(podL.getPole(), podR.getPole());
        posSystem.calculatePos();

//        reset.resetAuto(false);

//        kinematics.setTurnPID(constants.kpTurning, constants.kiTurning, constants.kdTurning);
        kinematics.setTurnPID(constants.kpTurning, constants.kiTurning, constants.kdTurning);

        kinematics.turn(finalAngle, speed);
        timeoutS = kinematics.getTimeOutTurn(finalAngle, speed);

        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

        robot.topL.setTargetPosition(targetTopL);
        robot.botL.setTargetPosition(targetBotL);
        robot.topR.setTargetPosition(targetTopR);
        robot.botR.setTargetPosition(targetBotR);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean turn = true;
        runTime.reset();
        while (turn && linearOpMode.opModeIsActive()){
            kinematics.turn(finalAngle, speed);

            targetClicks = kinematics.getClicks();
            motorPower = kinematics.getPower();

            powerL = motorPower[0];
            powerR = motorPower[2];

            targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
            targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
            targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
            targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.botL.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.topR.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.botR.setVelocity(speed * constants.MAX_VELOCITY_DT);

//            robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
//            robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
//            robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
//            robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);

            UpdateTelemetry();
            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();

            if (Math.abs(SwervePod.changeAngle(finalAngle, posSystem.getPositionArr()[4])) < constants.degreeTOLERANCE){
                if (stopConditionTimer.seconds() > 0.5){
                    turn = false;
                }
            } else {
                stopConditionTimer.reset();
            }

            if (runTime.seconds() > timeoutS){
//                turn = false;
//                break;
            }

        }

        reset();
    }

    public void Hone(CameraMaster detector, double speed){
        posSystem.resetXY(); // <-- Must have!
        posSystem.setPoles(podL.getPole(), podR.getPole());
        posSystem.calculatePos();

        double currentR = posSystem.getPositionArr()[4];
        double target = posSystem.clamp(currentR + detector.getAngle());

        Turn(target, speed);
    }

    public void fastHone(CameraMaster detector, double speed){
        posSystem.resetXY(); // <-- Must have!
        posSystem.setPoles(podL.getPole(), podR.getPole());
        posSystem.calculatePos();

        double currentR = posSystem.getPositionArr()[4];
        double target = posSystem.clamp(currentR + detector.getAngle());

        fastTurn(target, speed);
    }

    public void fastTurn(double finalAngle, double speed){
        robot.setInternalPIDFCoef(1, 0, 0, 0, 1, 0, 0, 0);

        //1) Calculate our current position
        posSystem.resetXY(); // <-- Must have!
        posSystem.setPoles(podL.getPole(), podR.getPole());
        posSystem.calculatePos();

//        reset.resetAuto(false);

//        kinematics.setTurnPID(constants.kpTurning, constants.kiTurning, constants.kdTurning);
        kinematics.setTurnPID(constants.kpTurning, constants.kiTurning, constants.kdTurning);

        kinematics.turn(finalAngle, speed);

        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

        robot.topL.setTargetPosition(targetTopL);
        robot.botL.setTargetPosition(targetBotL);
        robot.topR.setTargetPosition(targetTopR);
        robot.botR.setTargetPosition(targetBotR);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean turn = true;
        while (turn && linearOpMode.opModeIsActive()){
            kinematics.turn(finalAngle, speed);

            if (Math.abs(SwervePod.changeAngle(finalAngle, posSystem.getPositionArr()[4])) < 9){
                turn = false;
                robot.topL.setVelocity(0);
                robot.botL.setVelocity(0);
                robot.topR.setVelocity(0);
                robot.botR.setVelocity(0);
                robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                stopConditionTimer.reset();

                break;
            }

            targetClicks = kinematics.getClicks();
            motorPower = kinematics.getPower();

            powerL = motorPower[0];
            powerR = motorPower[2];

            targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
            targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
            targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
            targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.botL.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.topR.setVelocity(speed * constants.MAX_VELOCITY_DT);
            robot.botR.setVelocity(speed * constants.MAX_VELOCITY_DT);
//            robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
//            robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
//            robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
//            robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);

            UpdateTelemetry();
            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();
        }

        if (stopConditionTimer.seconds() < 1.0){
            robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        reset();
    } //faster, but sacrifices accuracy

    double[] armClicks = new double[5];
    public double[] getArmClicks(){
        armClicks[0] = robot.at.getCurrentPosition();
        armClicks[1] = robot.abl.getCurrentPosition();
        armClicks[2] = robot.abr.getCurrentPosition();
        armClicks[3] = robot.armServo.getPosition();
        armClicks[4] = robot.claw.getPosition();
        return armClicks;
    }

    public void run(){
        while (linearOpMode.opModeIsActive()){
            if (kinematics.isArmTargetMet()){
                robot.at.setTargetPosition(robot.at.getCurrentPosition());
                robot.abl.setTargetPosition(robot.abl.getCurrentPosition());
                robot.abr.setTargetPosition(robot.abr.getCurrentPosition());

                robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.at.setPower(0.4);
                robot.abl.setPower(1);
                robot.abr.setPower(1);
            }
            else if (!targetNotMet){
                robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    public void resetToZero(){
        robot.at.setTargetPosition(0);
        robot.abl.setTargetPosition(-constants.INIT_ARMBASE_POS + 10);
        robot.abr.setTargetPosition(-constants.INIT_ARMBASE_POS + 10);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setPower(0.5);
        robot.abl.setPower(0.5);
        robot.abr.setPower(0.5);

        robot.armServo.setPosition(0.4);
        robot.claw.setPosition(constants.openClaw);
    }

    public void resetGPS(){
        posSystem.resetXY();
        posSystem.resetHeader();
        posSystem.hardResetGPS();
    }

    public void claw(boolean open) throws InterruptedException { //use this only while the arm is in the air
        if (open){
            robot.armServo.setPosition(constants.clawDown);
            Thread.sleep(150); //assumes that nothing happens in 200 ms
            robot.claw.setPosition(constants.openClaw);
        } else {
            robot.armServo.setPosition(0.3);
            Thread.sleep(150);
            robot.claw.setPosition(constants.closeClaw);
        }
    }

    public void UpdateTelemetry(){
        linearOpMode.telemetry.addData("PIDF Coefficients topL", robot.topL.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        linearOpMode.telemetry.addData("PIDF Coefficients botL", robot.botL.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        linearOpMode.telemetry.addData("PIDF Coefficients topR", robot.topR.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        linearOpMode.telemetry.addData("PIDF Coefficients botR", robot.botR.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        linearOpMode.telemetry.addData("Wheel target met?", !targetNotMet);
        linearOpMode.telemetry.addData("Arm target met?", kinematics.isArmTargetMet());

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
//        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
//        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("Optimized Left W", podL.getOptimizedCurrentW());
        linearOpMode.telemetry.addData("Optimized Right W", podR.getOptimizedCurrentW());
//        linearOpMode.telemetry.addData("R reference point", podR.controlHeaderReference);
        linearOpMode.telemetry.addData("R", posSystem.getPositionArr()[4]);


        linearOpMode.telemetry.addData("InitPole L", podL.getPole());
        linearOpMode.telemetry.addData("InitPole R", podR.getPole());
//
        linearOpMode.telemetry.addData("target", kinematics.finalAngle);
        linearOpMode.telemetry.addData("Turn Amount (Left)", podL.getTurnAmount());
        linearOpMode.telemetry.addData("Turn Amount (Right)", podR.getTurnAmount());
        linearOpMode.telemetry.addData("Throttle (Left)", podL.getThrottle());
        linearOpMode.telemetry.addData("Throttle (Right)", podR.getThrottle());
//        linearOpMode.telemetry.addData("error L", podL.controlHeader.error);
//        linearOpMode.telemetry.addData("error R", podR.controlHeader.error);
//        linearOpMode.telemetry.addData("error L arc", podL.controlHeader.biggerArc);
//        linearOpMode.telemetry.addData("error R arc", podR.controlHeader.biggerArc);

        linearOpMode.telemetry.addData("DistanceL", podL.getDistance());
        linearOpMode.telemetry.addData("DistanceR", podR.getDistance());
        linearOpMode.telemetry.addData("Distance Ran R", posSystem.distanceTravelledR);
        linearOpMode.telemetry.addData("Distance Ran L", posSystem.distanceTravelledL);

        linearOpMode.telemetry.addData("Power TopL", robot.topL.getPower());
        linearOpMode.telemetry.addData("Power BotL", robot.botL.getPower());
        linearOpMode.telemetry.addData("Power TopR", robot.topR.getPower());
        linearOpMode.telemetry.addData("Power BotR", robot.botR.getPower());

        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
        linearOpMode.telemetry.addData("Arm Type", kinematics.getArmType());

        linearOpMode.telemetry.addData("Delta time loop (sec)", deltaMS / 1000.0);

//        packet.put("PowerR", powerR);
        packet.put("PowerL", powerL);
//        packet.put("Error R", podR.getPID().getError());
        packet.put("Error L", podL.getPID().getError());
//        packet.put("Distance Travelled R", posSystem.getDistanceTravelledR());
        packet.put("Distance Travelled L", posSystem.getDistanceTravelledL());
        packet.put("Distance Remaining (L)", podL.getDistance());
        packet.put("Distance Remaining (R)", podR.getDistance());

        packet.put("Left W",  posSystem.getLeftWheelW());
        packet.put("Robot Header", posSystem.getPositionArr()[4]);

//        packet.put("Error at", kinematics.atPID.getError());
//        packet.put("Target at", kinematics.atPID.getTarget());

//        packet.put("Error abl", kinematics.ablPID.getError());
//        packet.put("Target abl", kinematics.ablPID.getTarget());

//        packet.put("Error abr", kinematics.abrPID.getError());
//        packet.put("Target abr", kinematics.abrPID.getTarget());
        packet.put("TimeOut", timeoutS);


        packet.put("delta time", deltaMS / 1000.0);
        packet.put("InitPole R", podR.getPole());
        packet.put("InitPole L", podL.getPole());
        packet.put("Drive Type", kinematics.getDriveType());
        packet.put("Timer", timer.seconds());

//        packet.put("Right W", posSystem.getRightWheelW());
        dashboard.sendTelemetryPacket(packet);

        linearOpMode.telemetry.update();
    }

    public void arm(double xValue, double yValue, double clawAngle){
        boolean targetMet = triangle(xValue, yValue, clawAngle);

        while (!targetMet){
            targetMet = triangle(xValue, yValue, clawAngle);
        }
    }

    boolean triangle(double xvalue, double yvalue, double clawAngle){
        double z = Math.sqrt((xvalue*xvalue)+(yvalue*yvalue));

        if (z>810){
            double tempx = xvalue;
            double tempy = yvalue;
            xvalue = 810*(Math.sin(Math.atan(tempx/tempy)));
            yvalue = 810*(Math.cos(Math.atan(tempx/tempy)));
        }

        if (z<40){
            xvalue = 30;
            yvalue = 30;
        }

//        if (xvalue < -30){
//            xvalue = -30;
//        }
        if (yvalue < 0){
            yvalue = 0;
        }

        z = Math.sqrt((xvalue*xvalue)+(yvalue*yvalue));

        double topMotorAngle = Math.toDegrees(Math.acos(((botSegLength*botSegLength)+(topSegLength*topSegLength)-(z*z))/(2*(botSegLength)*(topSegLength))));
        double bottomMotorAngle = Math.toDegrees(Math.acos(((z*z)+(topSegLength*topSegLength)-(botSegLength*botSegLength))/(2*(z)*(topSegLength))));
        double finalBottomarmangle = 180-(bottomMotorAngle + Math.toDegrees(Math.asin(yvalue/z)));
        double servoAngleChange = (180 - (topMotorAngle-finalBottomarmangle-constants.offset))-90;

        servoUnits = 0.6-(servoAngleChange/constants.anglePerUnit) + clawAngle;
        int topMotorClicks = (int)((topMotorAngle/constants.topMotorAnglePerClick)-(constants.topMotorInitialAngle/constants.topMotorAnglePerClick));
        int bottomMotorClicks = (int)((finalBottomarmangle/constants.bottomMotorAnglePerClick)-(constants.bottomMotorInitialAngle/constants.bottomMotorAnglePerClick));

        if (servoUnits < 0.0){
            servoUnits = 0.1;
        }
        if (servoUnits > 1.0){
            servoUnits = 0.9;
        }

        if (xvalue>=0){
            robot.at.setTargetPosition(topMotorClicks);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower((30/76.0));

            robot.abl.setTargetPosition(bottomMotorClicks);
            robot.abr.setTargetPosition(bottomMotorClicks);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.7);
            robot.abr.setPower(0.7);
            robot.armServo.setPosition(servoUnits);
        }

        return(Math.abs(robot.at.getCurrentPosition() - topMotorClicks) < constants.clickTOLERANCE &&
                Math.abs(robot.abl.getCurrentPosition() - bottomMotorClicks) < constants.clickTOLERANCE &&
                Math.abs(robot.abr.getCurrentPosition() - bottomMotorClicks) < constants.clickTOLERANCE); //returns true when target met.  False otherwise.


//        if ((xvalue < 0) && (topMotorAngle > 150)){
//            double addnumber = 0.85 * Math.abs(xvalue);
//            robot.at.setTargetPosition(920);
//            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.at.setPower((30/76.0));
//
//            robot.abl.setTargetPosition(520 - (int)addnumber);
//            robot.abr.setTargetPosition(520 - (int)addnumber);
//            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.abl.setPower(0.7);
//            robot.abr.setPower(0.7);
//            robot.armServo.setPosition(servoUnits);
//        }
    }
}
