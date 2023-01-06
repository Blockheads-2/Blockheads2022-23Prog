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
import org.firstinspires.ftc.teamcode.common.kinematics.AutoKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;

public class AutoHubJR {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;
    Reset reset;

    double[] motorPower = new double[4];
    int[] targetClicks = new int[4];
    boolean targetNotMet = true;

    View relativeLayout;

    public AutoHubJR(LinearOpMode plinear){
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
    }

    public void Move(RevisedKinematics.DriveType movementType, double x, double y, double finalAngle, double speed){
        UpdateTelemetry();

        //1) Calculate our current position
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        kinematics.logicAuto();

        reset.reset(false);

        targetNotMet = true;
        targetClicks = kinematics.getClicks();
        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];


        //3) Tell the robot to travel that distance we just determined.
        while (linearOpMode.opModeIsActive() && targetNotMet){ //have a time based something in case our target is never met.
            posSystem.calculatePos();

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorPower = kinematics.getPower();
            robot.topL.setPower(motorPower[0]);
            robot.botL.setPower(motorPower[1]);
            robot.topR.setPower(motorPower[2]);
            robot.botR.setPower(motorPower[3]);

            targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > constants.degreeTOLERANCE || Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.degreeTOLERANCE || Math.abs(robot.topR.getCurrentPosition() - targetTopR) > constants.degreeTOLERANCE || Math.abs(robot.botR.getCurrentPosition() - targetBotR) > constants.degreeTOLERANCE);
            UpdateTelemetry();
        }

//        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
//            while(reset.finishedReset()){
//                reset.reset(true);
//            }
//        }
    }

    public void UpdateTelemetry(){
        linearOpMode.telemetry.addData("Target not met", targetNotMet);
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

        linearOpMode.telemetry.update();
    }
}
