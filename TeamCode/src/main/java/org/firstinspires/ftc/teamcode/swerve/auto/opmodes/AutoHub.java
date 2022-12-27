package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;

public class AutoHub {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;

    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();

    View relativeLayout;

    public enum DriveType{
        LINEAR,
        SPLINE,
        TURN_ON_CENTER,
    }

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        posSystem = new GlobalPosSystem(robot);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
        linearOpMode.telemetry.update();

    }

    //for now, we'll leave it as last year's template, but it might be a good idea to be constantly calculating its position and
    // how much it needs to go to lessen error.
    void Move(DriveType movementType, double x, double y, boolean linear, double finalAngle, double speed){

    }
}
