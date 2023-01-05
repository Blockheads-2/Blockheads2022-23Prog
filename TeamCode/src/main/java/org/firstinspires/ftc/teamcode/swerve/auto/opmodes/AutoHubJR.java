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

    View relativeLayout;

    public AutoHubJR(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        posSystem = new GlobalPosSystem(robot);
        kinematics = new RevisedKinematics(posSystem);
        reset = new Reset(robot, posSystem);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
        linearOpMode.telemetry.update();
    }

    public void Move(RevisedKinematics.DriveType movementType, double x, double y, double finalAngle, double speed){
        //1) Calculate our current position
        posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
        kinematics.setPosAuto(x, y, finalAngle, speed, movementType);
        kinematics.logicAuto();

        reset.reset(false);

        boolean targetNotMet = true;
        int[] targetClicks = kinematics.getClicks();
        int targetTopL = robot.topL.getCurrentPosition() + targetClicks[0];
        int targetBotL = robot.botL.getCurrentPosition() + targetClicks[1];
        int targetTopR = robot.topR.getCurrentPosition() + targetClicks[2];
        int targetBotR = robot.botR.getCurrentPosition() + targetClicks[3];


        //3) Tell the robot to travel that distance we just determined.
        while (linearOpMode.opModeIsActive() && targetNotMet){
            posSystem.calculatePos();

            robot.topL.setTargetPosition(targetTopL);
            robot.botL.setTargetPosition(targetBotL);
            robot.topR.setTargetPosition(targetTopR);
            robot.botR.setTargetPosition(targetBotR);

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double[] motorPower = kinematics.getPower();
            robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
            robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
            robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
            robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);

            targetNotMet = (Math.abs(robot.topL.getCurrentPosition() - targetTopL) > constants.clickTOLERANCE && Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.clickTOLERANCE && Math.abs(robot.topR.getCurrentPosition() - targetTopR) > constants.clickTOLERANCE && Math.abs(robot.botL.getCurrentPosition() - targetBotL) > constants.clickTOLERANCE);
        }

        if (kinematics.getDriveType() != RevisedKinematics.DriveType.SNAP){
            while(reset.finishedReset()){
                reset.reset(true);
            }
        }
    }

    public void telemetryData(){

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("R", posSystem.getPositionArr()[4]);

//        linearOpMode.telemetry.addData("Spin Direction (Left)", kinematics.params.get("throttleL"));
//        linearOpMode.telemetry.addData("Spin Direction (Right)", kinematics.params.get("throttleR"));

        linearOpMode.telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        linearOpMode.telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        linearOpMode.telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        linearOpMode.telemetry.addData("botR clicks", robot.botR.getCurrentPosition());
//        linearOpMode.telemetry.addData("Left Rotate Power", kinematics.output.get("rotPowerL"));
//        linearOpMode.telemetry.addData("Right Rotate Power", kinematics.output.get("rotPowerR"));

//        linearOpMode.telemetry.addData("Right Clicks target", kinematics.output.get("spinClicksR") + kinematics.output.get("rotClicksR"));

//        linearOpMode.telemetry.addData("Left Spin Power", kinematics.output.get("spinPowerL"));
//        linearOpMode.telemetry.addData("Right Spin Power", kinematics.output.get("spinPowerL"));
        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
//        linearOpMode.telemetry.addData("First movement", kinematics.firstMovement);

        linearOpMode.telemetry.update();
    }
}
