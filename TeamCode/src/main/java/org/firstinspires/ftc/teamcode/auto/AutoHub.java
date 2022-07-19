package org.firstinspires.ftc.teamcode.auto;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.kinematics.LinearMotion;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

public class AutoHub {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    LinearMotion linearMotion = new LinearMotion();
    GlobalPosSystem posSystem = new GlobalPosSystem();

    View relativeLayout;

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
        linearOpMode.telemetry.update();
    }


    public void linearMovement(double x, double y, double turnDegrees, double kp, double ki, double kd){
        posSystem.calculatePos();
        linearMotion.setPos(x, y, turnDegrees, posSystem.getPositionArr()[2], posSystem.getPositionArr()[3]);
        snap(turnDegrees);

        int[] encoderTargets = new int[4];

        for (int i = 0; i < 4; i++){
            encoderTargets[i] = linearMotion.getClicks();
            robot.dtMotors[i].setTargetPosition(encoderTargets[i]);
        }

        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (linearOpMode.opModeIsActive() && robot.dtMotors[0].isBusy() && robot.dtMotors[1].isBusy() && robot.dtMotors[2].isBusy() && robot.dtMotors[3].isBusy()){
            posSystem.calculatePos();
            for (int i = 0; i < 3; i++){
                robot.dtMotors[i].setVelocity(linearMotion.getVelocity()[i]);
            }
        }
        robot.setMotorPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void snap(double turnDegrees){
        double targetOrientation = turnDegrees + posSystem.getPositionArr()[2];
        double currentOrientation = posSystem.getPositionArr()[2];
        RotateSwerveModulePID rotatePID = new RotateSwerveModulePID(targetOrientation, 0.0, 0.0, 0.0);
        while (Math.abs(currentOrientation - targetOrientation) <= constants.TOLERANCE){
            posSystem.calculatePos();
            double power = rotatePID.update(currentOrientation);
            robot.dtMotors[0].setPower(power);
            robot.dtMotors[1].setPower(-power);
            robot.dtMotors[2].setPower(power);
            robot.dtMotors[3].setPower(-power);
        }
        robot.setMotorPower(0);
    }


    public void spline(double x, double y, double turnDegrees, double kp, double ki, double kd){

    }
}
