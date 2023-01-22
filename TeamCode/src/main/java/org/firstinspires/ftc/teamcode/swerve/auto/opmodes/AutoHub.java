package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.swerve.common.Accelerator;
import org.firstinspires.ftc.teamcode.swerve.common.Reset;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;
import org.firstinspires.ftc.teamcode.swerve.common.pid.HeaderControlPID;
import org.firstinspires.ftc.teamcode.swerve.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

public class AutoHub implements Runnable{
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;
    Reset reset;

    Accelerator accelerator = new Accelerator();
    SwervePod podR;
    SwervePod podL;
    HeaderControlPID controlHeader;


    TurnMath turnMath = new TurnMath();
    SnapSwerveModulePID turnPID = new SnapSwerveModulePID(); //although it says Snap, it works for turning as well.

    double[] motorPower = new double[4];
    int[] targetClicks = new int[4];
    boolean targetNotMet = true;
    double[] armOutput = new double[8];

    View relativeLayout;

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);

        accelerator.actuallyAccelerate(true);
        accelerator.setAccelFactor(constants.accelTimeAuto);
        podR = new SwervePod(constants.initDirectionRight, SwervePod.Side.RIGHT, accelerator);
        podL = new SwervePod(constants.initDirectionLeft, SwervePod.Side.LEFT, accelerator);

        kinematics = new RevisedKinematics(posSystem, podL, podR);
        posSystem.grabKinematics(kinematics);
        kinematics.grabTelemetry(linearOpMode.telemetry);
        reset = new Reset(robot, posSystem);
        controlHeader = new HeaderControlPID(posSystem.getMotorClicks());
        podR.setHeaderController(controlHeader);
        podL.setHeaderController(controlHeader);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

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

    public void Move(RevisedKinematics.DriveType movementType, double x, double y, double finalAngle, double speed, RevisedKinematics.ArmType armMovementType){
        UpdateTelemetry();

        //1) Calculate our current position
        posSystem.resetXY();
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        reset.resetAuto(false);
        kinematics.armLogicAuto(armMovementType); //determine targets/power for the arm
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

        //3) Tell the robot to travel that distance we just determined.
        while (linearOpMode.opModeIsActive() && (targetNotMet || !kinematics.isArmTargetMet())){ //have a time based something in case our target is never met.
            posSystem.calculatePos();
            kinematics.armLogicAuto(armMovementType); //determine targets/power for the arm
            kinematics.logicAuto();

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

            robot.topL.setPower(motorPower[0]);
            robot.botL.setPower(motorPower[1]);
            robot.topR.setPower(motorPower[2]);
            robot.botR.setPower(motorPower[3]);

            robot.at.setPower(armOutput[0]);
            robot.abl.setPower(armOutput[1]);
            robot.abr.setPower(armOutput[2]);

            targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > constants.clickToleranceAuto ||
                    Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.clickToleranceAuto ||
                    Math.abs(robot.topR.getCurrentPosition() - targetTopR) > constants.clickToleranceAuto ||
                    Math.abs(robot.botR.getCurrentPosition() - targetBotR) > constants.clickToleranceAuto);
            UpdateTelemetry();
        }

        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
            while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
                reset.resetAuto(true);
                linearOpMode.telemetry.addData("RESET", true);
            }
        } else {
            robot.topL.setTargetPosition(robot.topL.getCurrentPosition());
            robot.botL.setTargetPosition(robot.botL.getCurrentPosition());
            robot.topR.setTargetPosition(robot.topR.getCurrentPosition());
            robot.botR.setTargetPosition(robot.botR.getCurrentPosition());
        }
        reset.resetAuto(false);
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

//    public void Turn(double turnAmount, double speed){ //needs to be integrated into RevisedKinematics
//        posSystem.calculatePos();
//        double initCurrentL = posSystem.getLeftWheelW();
//        double initCurrentR = posSystem.getRightWheelW();
//
//        reset.resetAuto(false);
//
//        double initAngle = kinematics.clamp(posSystem.getPositionArr()[4]);
//
//        turnMath.setPos(turnAmount, posSystem.getMotorClicks()[0], posSystem.getMotorClicks()[2], (turnAmount < 0 ? -1 : 1));
//
//        turnPID.setTargets(0.03, 0, 0.03);
//
//        int direction = (turnAmount <= 0 ? -1 : 1);
//
//        double target = kinematics.clamp(initAngle + turnAmount);
//        int distanceR = turnMath.getTargetClicks();
//        int distanceL = turnMath.getTargetClicks();
//
//        //keep wheels parallel:
//        //...
//
//        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + (distanceL * direction * (int)constants.initDirectionLeft));
//        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - (distanceL * direction * (int)constants.initDirectionLeft));
//        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + (distanceR * direction * (int)constants.initDirectionRight));
//        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - (distanceR * direction * (int)constants.initDirectionRight));
//
//        while (Math.abs(target - posSystem.getPositionArr()[4]) >= constants.degreeTOLERANCE && linearOpMode.opModeIsActive()){
//            posSystem.calculatePos();
//
//            double power = turnPID.update(turnMath.getAngleRemaining(posSystem.getPositionArr()[4]));
//
//            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.topL.setPower(power * speed);
//            robot.botL.setPower(power * speed);
//            robot.topR.setPower(power * speed);
//            robot.botR.setPower(power * speed);
//
//            UpdateTelemetry();
//        }
//
//        while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
//            UpdateTelemetry();
//            reset.resetAuto(true);
//        }
//        reset.resetAuto(false);
//
//    }

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

    public void UpdateTelemetry(){
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

        linearOpMode.telemetry.addData("Spin Direction (Left)", podL.getSpinDirection());
        linearOpMode.telemetry.addData("Spin Direction (Right)", podR.getSpinDirection());
//
        linearOpMode.telemetry.addData("target", kinematics.finalAngle);
        linearOpMode.telemetry.addData("Turn Amount (Left)", podL.getTurnAmount());
        linearOpMode.telemetry.addData("Turn Amount (Right)", podR.getTurnAmount());
        linearOpMode.telemetry.addData("Throttle (Left)", podL.getThrottle());
        linearOpMode.telemetry.addData("Throttle (Right)", podR.getThrottle());
        linearOpMode.telemetry.addData("error L", podL.controlHeader.error);
        linearOpMode.telemetry.addData("error R", podR.controlHeader.error);
        linearOpMode.telemetry.addData("error L arc", podL.controlHeader.biggerArc);
        linearOpMode.telemetry.addData("error R arc", podR.controlHeader.biggerArc);

        linearOpMode.telemetry.addData("DistanceL", podL.getDistance());
        linearOpMode.telemetry.addData("DistanceR", podR.getDistance());
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

        linearOpMode.telemetry.update();
    }
}
