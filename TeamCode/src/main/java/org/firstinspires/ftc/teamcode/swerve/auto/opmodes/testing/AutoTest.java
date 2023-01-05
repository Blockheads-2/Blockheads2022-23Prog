package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.AutoKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHubJR;

@Autonomous (name = "Auto Test", group = "Drive")
public class AutoTest extends LinearOpMode {

    Constants constants = new Constants();
    AutoHubJR dispatch;

    View relativeLayout;



    @Override
    public void runOpMode() throws InterruptedException {

        dispatch = new AutoHubJR(this);

        telemetry.addData("Status", "Waiting on Camera");
        telemetry.update();

        waitForStart();

        dispatch.Move(RevisedKinematics.DriveType.SNAP, 10, 10, 0, 0.7);
        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 10, 10, 0, 0.7);
    }
}
