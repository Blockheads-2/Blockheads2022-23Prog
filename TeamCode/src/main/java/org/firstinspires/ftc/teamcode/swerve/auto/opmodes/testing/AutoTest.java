package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;

@Autonomous (name = "Auto Test", group = "Drive")
public class AutoTest extends LinearOpMode {

    Constants constants = new Constants();
    AutoHub dispatch;
    GlobalPosSystem posSystem;

    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        telemetry.addData("Status", "Waiting on Camera");
        telemetry.update();


        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)
        }

        waitForStart();

        Thread rigidArmThread = new Thread(dispatch);   // Using the constructor Thread(Runnable r)
        rigidArmThread.start();

//        Thread gpsUpdateThread = new Thread(posSystem);
//        gpsUpdateThread.start();

        dispatch.Move(RevisedKinematics.DriveType.SNAP, 0, 0, 45, 0.4, RevisedKinematics.ArmType.HOLD);
        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, constants.WHEEL_CIRCUMFERENCE, 0, 0.4, RevisedKinematics.ArmType.HIGH);
        dispatch.Move(RevisedKinematics.DriveType.SNAP, 0, 0, -90, 0.4, RevisedKinematics.ArmType.HOLD);
        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, 27, 0, 0.4, RevisedKinematics.ArmType.HOLD);

        /*
        working movement types:
        - SNAP
        - LINEAR
        - STOP
        - Arm movements
         */

//        dispatch.Move(RevisedKinematics.DriveType.STOP, 0, 0, 0, 0, RevisedKinematics.ArmType.LOW);
//        dispatch.Turn(90, 0.7);

        rigidArmThread.interrupt();
//        gpsUpdateThread.interrupt();

    }
}
