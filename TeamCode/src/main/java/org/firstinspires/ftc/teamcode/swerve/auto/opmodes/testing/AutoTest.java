package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.swerve.common.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;
import org.opencv.objdetect.HOGDescriptor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Auto Test", group = "Drive")
public class AutoTest extends LinearOpMode {

    OpenCvCamera camera;
    Constants constants = new Constants();
    AutoHub dispatch;
    GlobalPosSystem posSystem;

    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);
        sleep(20);

//        if (!opModeIsActive()) dispatch.moveToInit();

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)
            dispatch.UpdateTelemetry();
        }
//        dispatch.resetArmEncoderPos();

        waitForStart();

        dispatch.resetToZero();

//        Thread rigidArmThread = new Thread(dispatch);   // Using the constructor Thread(Runnable r)
//        rigidArmThread.start();

//        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, 40, 0, 0.5, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);

//
//        dispatch.Move(RevisedKinematics.DriveType.SNAP, 0, 0, AutoHub.finalSnapAngle, AutoHub.powerRotate, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
//        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, AutoHub.distance, 0, AutoHub.powerTranslate, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
//        dispatch.fastTurn(AutoHub.finalTurnAngle, AutoHub.powerTurn);
//
//        dispatch.Turn(AutoHub.finalTurnAngle, AutoHub.powerTurn);

//        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, AutoHub.distance2, 0, AutoHub.powerTranslate, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
//        dispatch.Move(RevisedKinematics.DriveType.STOP, 0, 0, 0, 0, RevisedKinematics.ArmType.HOLD, constants.armServoMid, constants.openClaw);

        dispatch.Move(RevisedKinematics.DriveType.SNAP, 0, 0, 135, 0.6, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
        dispatch.fullReset();


//        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, 30, 0, 0.6, RevisedKinematics.ArmType.MID, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
//        dispatch.Turn(135, 0.3);
//        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, 6, 0, 0.6, RevisedKinematics.ArmType.HOLD, dispatch.getArmClicks()[3], dispatch.getArmClicks()[4]);
//        dispatch.Move(RevisedKinematics.DriveType.STOP, 0, 0, 0, 0, RevisedKinematics.ArmType.HOLD, constants.armServoMid, constants.openClaw);

        /*
        working movement types:
        - SNAP
        - LINEAR
        - STOP
        - Arm movements
         */

//        dispatch.Move(RevisedKinematics.DriveType.STOP, 0, 0, 0, 0, RevisedKinematics.ArmType.LOW);
//        dispatch.Turn(90, 0.7);

//        dispatch.resetToZero();
//        rigidArmThread.interrupt();
//        gpsUpdateThread.interrupt();

        }
    }

