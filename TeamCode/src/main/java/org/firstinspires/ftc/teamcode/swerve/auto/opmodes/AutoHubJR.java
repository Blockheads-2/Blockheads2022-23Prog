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

public class AutoHubJR {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    AutoKinematics kinematics;
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
        kinematics = new AutoKinematics(posSystem);
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

    public void Move(AutoKinematics.DriveType movementType, double x, double y, double finalAngle, double speed){
        posSystem.calculatePos();
        kinematics.setDriveType(movementType);

        kinematics.setPos(x, y, finalAngle, speed);
        reset.reset(false);

        while (linearOpMode.opModeIsActive() && robot.wheelsAreBusy()){
        //1) Calculate our current position
            posSystem.calculatePos();

        //2) Determine the distance from our current pos & the target pos.
//            kinematics.setPos(x, y, finalAngle, speed);
            kinematics.logic();

        //3) Tell the robot to travel that distance we just determined.
            int[] targetClicks = kinematics.getClicks();
            robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
            robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
            robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
            robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double[] motorPower = kinematics.getPower();
            robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
            robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
            robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
            robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);
        }

        while(reset.finishedReset()){
            reset.reset(true);
        }
    }

    public void telemetryData(){

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("R", posSystem.getPositionArr()[4]);

        linearOpMode.telemetry.addData("Spin Direction (Left)", kinematics.params.get("throttleL"));
        linearOpMode.telemetry.addData("Spin Direction (Right)", kinematics.params.get("throttleR"));

        linearOpMode.telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        linearOpMode.telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        linearOpMode.telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        linearOpMode.telemetry.addData("botR clicks", robot.botR.getCurrentPosition());
        linearOpMode.telemetry.addData("Left Rotate Power", kinematics.output.get("rotPowerL"));
        linearOpMode.telemetry.addData("Right Rotate Power", kinematics.output.get("rotPowerR"));

        linearOpMode.telemetry.addData("Right Clicks target", kinematics.output.get("spinClicksR") + kinematics.output.get("rotClicksR"));

        linearOpMode.telemetry.addData("Left Spin Power", kinematics.output.get("spinPowerL"));
        linearOpMode.telemetry.addData("Right Spin Power", kinematics.output.get("spinPowerL"));
        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
        linearOpMode.telemetry.addData("First movement", kinematics.firstMovement);

        linearOpMode.telemetry.update();
    }
}
