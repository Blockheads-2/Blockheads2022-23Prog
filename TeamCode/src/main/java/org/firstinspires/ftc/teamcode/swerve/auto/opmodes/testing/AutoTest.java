package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.ArmAuto;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;

@Autonomous (name = "Auto Test", group = "Drive")
public class AutoTest extends LinearOpMode {

    Constants constants = new Constants();
    AutoHub dispatch;

    View relativeLayout;



    @Override
    public void runOpMode() throws InterruptedException {

        dispatch = new AutoHub(this);

        telemetry.addData("Status", "Waiting on Camera");
        telemetry.update();

        waitForStart();

        dispatch.Move(RevisedKinematics.DriveType.SNAP, 10, 10, 0, 0.7, ArmAuto.ArmType.NONE);
        dispatch.Move(RevisedKinematics.DriveType.LINEAR, 0, constants.WHEEL_CIRCUMFERENCE, 0, 0.7, ArmAuto.ArmType.HIGH);
    }
}
