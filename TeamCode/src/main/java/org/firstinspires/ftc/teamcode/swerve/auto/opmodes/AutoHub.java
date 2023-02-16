package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.swerve.common.Button;
import org.firstinspires.ftc.teamcode.swerve.common.Reset;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;
import org.firstinspires.ftc.teamcode.swerve.common.pid.HeaderControlPID;
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

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double power = 0.5;
    public static double distance = 0;
    public static double finalAngleDashboard = 0;
    public static double finalSnapAngle = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime loopTime = new ElapsedTime();
    double prevMS = 0;
    double deltaMS = 0;

    ElapsedTime stopConditionTimer = new ElapsedTime();

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

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToInit(){

        robot.abl.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.abr.setTargetPosition(constants.INIT_ARMBASE_POS);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(0.6);
        robot.abr.setPower(0.6);
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
        UpdateTelemetry();

        //1) Calculate our current position
        posSystem.resetXY(); // <-- Must have!
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        timeoutS = kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        //        timeoutS = 100;
        reset.resetAuto(false);
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
        robot.armServo.setPosition(armOutput[6]);
        robot.claw.setPosition(armOutput[7]);


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
        while (linearOpMode.opModeIsActive() && (targetNotMet || !armTargetMet) && runTime.seconds() < timeoutS + constants.autoStopConditionTime + 3){ //have a time based something in case our target is never met.
            posSystem.calculatePos();
            kinematics.armLogicAuto(armMovementType, getArmClicks(), clawAngle, claw); //determine targets/power for the arm
            // see how long program takes without calling armLogicAuto

            kinematics.logicAuto();

            targetClicks = kinematics.getClicks();
            motorPower = kinematics.getPower();

            powerL = motorPower[0];
            powerR = motorPower[2];

//            targetClicks[2] = 0;
//            targetClicks[3] = 0;
//            motorPower[2] = 0;
//            motorPower[3] = 0;

            targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
            targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
            targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
            targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setPower(motorPower[0]);
            robot.botL.setPower(motorPower[1]);
            robot.topR.setPower(motorPower[2]);
            robot.botR.setPower(motorPower[3]);

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

            if (!targetNotMet && stopConditionTimer.seconds() > constants.autoStopConditionTime){
                targetNotMet = false;
            } else {
                stopConditionTimer.reset();
                targetNotMet = true;
            }

//            armTargetMet = false;

            UpdateTelemetry();

            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();
        }

        podL.getAccelerator().resetAccelerationAuto();
        podL.getAccelerator().resetAccelerationAuto();

        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
            while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
                reset.resetAuto(true);
                linearOpMode.telemetry.addData("RESET", true);
                deltaMS = loopTime.milliseconds() - prevMS;
                prevMS = loopTime.milliseconds();
            }
        } else {
            robot.topL.setTargetPosition(robot.topL.getCurrentPosition());
            robot.botL.setTargetPosition(robot.botL.getCurrentPosition());
            robot.topR.setTargetPosition(robot.topR.getCurrentPosition());
            robot.botR.setTargetPosition(robot.botR.getCurrentPosition());
        }
        reset.resetAuto(false);

        robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    public void Turn(double finalAngle, double speed){
        //1) Calculate our current position
        posSystem.resetXY(); // <-- Must have!
        posSystem.calculatePos();

        reset.resetAuto(false);

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
//        while (linearOpMode.opModeIsActive()){
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

            robot.topL.setPower(motorPower[0]);
            robot.botL.setPower(motorPower[1]);
            robot.topR.setPower(motorPower[2]);
            robot.botR.setPower(motorPower[3]);

            UpdateTelemetry();
            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();


            if (Math.abs(SwervePod.changeAngle(finalAngle, posSystem.getPositionArr()[4])) < constants.degreeTOLERANCE){
                if (stopConditionTimer.seconds() > 2.0){

                    turn = false;
                }
            } else {
                stopConditionTimer.reset();
            }
        }

        robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
            reset.resetAuto(true);
            linearOpMode.telemetry.addData("RESET", true);
            deltaMS = loopTime.milliseconds() - prevMS;
            prevMS = loopTime.milliseconds();
        }

        reset.resetAuto(false);

        robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

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
        robot.abl.setTargetPosition(0);
        robot.abr.setTargetPosition(0);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setPower(0.4);
        robot.abl.setPower(1);
        robot.abr.setPower(1);
    }

    public void clawAngle(double servoVal){
        robot.armServo.setPosition(servoVal);
    }
    public void UpdateTelemetry(){
        linearOpMode.telemetry.addData("KP", kp);
        linearOpMode.telemetry.addData("KI", ki);
        linearOpMode.telemetry.addData("KD", kd);
        linearOpMode.telemetry.addData("Wheel target met?", !targetNotMet);
        linearOpMode.telemetry.addData("Arm target met?", kinematics.isArmTargetMet());

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("Optimized Left W", podL.getOptimizedCurrentW());
        linearOpMode.telemetry.addData("Optimized Right W", podR.getOptimizedCurrentW());
        linearOpMode.telemetry.addData("R reference point", podR.controlHeaderReference);
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

        linearOpMode.telemetry.addData("TurnAmountL", podL.getTurnAmount());
        linearOpMode.telemetry.addData("TurnAmountR", podR.getTurnAmount());

        linearOpMode.telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        linearOpMode.telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        linearOpMode.telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        linearOpMode.telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        linearOpMode.telemetry.addData("Power TopL", motorPower[0]);
        linearOpMode.telemetry.addData("Power BotL", motorPower[1]);
        linearOpMode.telemetry.addData("Power TopR", motorPower[2]);
        linearOpMode.telemetry.addData("Power BotR", motorPower[3]);
        linearOpMode.telemetry.addData("Clicks Target TopL", targetClicks[0]);
        linearOpMode.telemetry.addData("Clicks Target BotL", targetClicks[1]);
        linearOpMode.telemetry.addData("Clicks Target TopR", targetClicks[2]);
        linearOpMode.telemetry.addData("Clicks Target BotR", targetClicks[3]);

        linearOpMode.telemetry.addData("Rot clicks L", podL.rotClicksTarget);
        linearOpMode.telemetry.addData("Rot clicks R", podR.rotClicksTarget);
        linearOpMode.telemetry.addData("spin clicks L", podL.spinClicksTarget);
        linearOpMode.telemetry.addData("spin clicks R", podR.spinClicksTarget);


        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
        linearOpMode.telemetry.addData("Arm Type", kinematics.getArmType());

        linearOpMode.telemetry.addData("Delta time loop (sec)", deltaMS / 1000.0);

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
//        packet.put("Right W", posSystem.getRightWheelW());
        dashboard.sendTelemetryPacket(packet);

        linearOpMode.telemetry.update();
    }
}
