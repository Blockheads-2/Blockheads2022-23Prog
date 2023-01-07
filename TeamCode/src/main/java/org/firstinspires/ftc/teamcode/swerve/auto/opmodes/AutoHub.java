package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

import java.io.BufferedReader;
import java.util.HashMap;

public class AutoHub implements Runnable{
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;
    Reset reset;

    TurnMath turnMath = new TurnMath();
    SnapSwerveModulePID turnPID = new SnapSwerveModulePID(); //although it says Snap, it works for turning as well.

    double[] motorPower = new double[4];
    int[] targetClicks = new int[4];
    boolean targetNotMet = true;
    HashMap<String, Double> armOutput = new HashMap<String, Double>();

    View relativeLayout;

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        posSystem = new GlobalPosSystem(robot);
        kinematics = new RevisedKinematics(posSystem);
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);

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

    public void Move(RevisedKinematics.DriveType movementType, double x, double y, double finalAngle, double speed, RevisedKinematics.ArmType armMovementType){
        UpdateTelemetry();

        //1) Calculate our current position
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        kinematics.logicAuto();

        reset.resetAuto(false);

        targetNotMet = true;
        targetClicks = kinematics.getClicks();
        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];

        armOutput = kinematics.getArmOutput();

        //3) Tell the robot to travel that distance we just determined.
        while (linearOpMode.opModeIsActive() && (targetNotMet || !kinematics.isArmTargetMet())){ //have a time based something in case our target is never met.
            posSystem.calculatePos();

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            kinematics.armLogicAuto(armMovementType); //determine targets/power for the arm

            robot.at.setTargetPosition(armOutput.get("atTargetPos").intValue());
            robot.abl.setTargetPosition(armOutput.get("ablTargetPos").intValue());
            robot.abr.setTargetPosition(armOutput.get("abrTargetPos").intValue());

            robot.armServo.setPosition(armOutput.get("clawAnglePos"));
            robot.claw.setPosition(armOutput.get("clawClampPos"));

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorPower = kinematics.getPower();
            robot.topL.setPower(motorPower[0]);
            robot.botL.setPower(motorPower[1]);
            robot.topR.setPower(motorPower[2]);
            robot.botR.setPower(motorPower[3]);

            robot.at.setPower(armOutput.get("atPower"));
            robot.abl.setPower(armOutput.get("ablPower"));
            robot.abr.setPower(armOutput.get("abrPower"));

            targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > constants.clickTOLERANCE ||
                    Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.clickTOLERANCE ||
                    Math.abs(robot.topR.getCurrentPosition() - targetTopR) > constants.clickTOLERANCE ||
                    Math.abs(robot.botR.getCurrentPosition() - targetBotR) > constants.clickTOLERANCE);
            UpdateTelemetry();
        }

        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
            while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
                reset.resetAuto(true);
            }
        }
        reset.resetAuto(false);
    }

    public void run(){
        while (linearOpMode.opModeIsActive()){
            if (kinematics.isArmTargetMet()){
                robot.at.setTargetPosition(robot.at.getCurrentPosition());
                robot.abl.setTargetPosition(robot.abl.getCurrentPosition());
                robot.abr.setTargetPosition(robot.abr.getCurrentPosition());

                robot.at.setPower(0.4);
                robot.abl.setPower(1);
                robot.abr.setPower(1);
            } else if (!targetNotMet){
                robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    public void Turn(double turnAmount, double speed){
        posSystem.calculatePos();
        reset.resetAuto(false);


        double initAngle = kinematics.clamp(posSystem.getPositionArr()[4]);

        turnMath.setPos(turnAmount, constants.kp, constants.ki, constants.kd);

        turnPID.setTargets(0.03, 0, 0.03);

        int direction = (turnAmount <= 0 ? -1 : 1);

        double target = kinematics.clamp(initAngle + turnAmount);

        int distance = (int)(turnMath.getDistance() * constants.CLICKS_PER_INCH);

        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distance * direction * (int)constants.initDirectionLeft);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - distance * direction * (int)constants.initDirectionLeft);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() - distance * direction * (int)constants.initDirectionRight);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + distance * direction * (int)constants.initDirectionRight);

        while (Math.abs(target - posSystem.getPositionArr()[4]) >= constants.degreeTOLERANCE && linearOpMode.opModeIsActive()){
            posSystem.calculatePos();

            double power = turnPID.update(turnMath.getDistanceRemaining(posSystem.getPositionArr()[4]));

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.topL.setPower(power * speed);
            robot.botL.setPower(power * speed);
            robot.topR.setPower(power * speed);
            robot.botR.setPower(power * speed);

            UpdateTelemetry();
        }

        while(!reset.finishedReset() && linearOpMode.opModeIsActive()){
            UpdateTelemetry();
            reset.resetAuto(true);
        }
        reset.resetAuto(false);

    }

    public void UpdateTelemetry(){
        linearOpMode.telemetry.addData("Wheel target met?", !targetNotMet);
        linearOpMode.telemetry.addData("Arm target met?", kinematics.isArmTargetMet());
        linearOpMode.telemetry.addData("Lower to mid?", kinematics.lowerArmCycle);
        linearOpMode.telemetry.addData("Lower to Bottom?", kinematics.lowerAllTheWay);

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("R", posSystem.getPositionArr()[4]);

        linearOpMode.telemetry.addData("Turn Amount (Left)", kinematics.turnAmountL);
        linearOpMode.telemetry.addData("Turn Amount (Right)", kinematics.turnAmountR);
        linearOpMode.telemetry.addData("DistanceL", kinematics.distanceL);
        linearOpMode.telemetry.addData("DistanceR", kinematics.distanceR);
        linearOpMode.telemetry.addData("TurnAmountL", kinematics.turnAmountL);
        linearOpMode.telemetry.addData("TurnAmountR", kinematics.turnAmountR);
        linearOpMode.telemetry.addData("Target", kinematics.target);

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

        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
        linearOpMode.telemetry.addData("Arm Type", kinematics.getArmType());

        linearOpMode.telemetry.update();
    }
}
