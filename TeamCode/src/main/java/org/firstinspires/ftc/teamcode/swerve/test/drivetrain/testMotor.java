package org.firstinspires.ftc.teamcode.swerve.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Test Motor", group="Drive")
//@Disabled
public class testMotor extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    public enum DriveType{
        CONTROLLER,
        TEST_AB
    }
    DriveType dType = DriveType.TEST_AB;
    boolean testAB = false;

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"

    }

    @Override
    public void start() { //When "start" is pressed
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        if(x.getState() == Button.State.TAP){
            dType = DriveType.CONTROLLER;
        } else if (y.getState() == Button.State.TAP){
            dType = DriveType.TEST_AB;
            testAB = true;
        }

        if (dType == DriveType.CONTROLLER){
            DriveTrainPowerEncoder();
        } else if (dType == DriveType.TEST_AB){
            testAB();
            testAB = false;
        }

    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }


    void DriveTrainPowerEncoder(){
        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();
        int posBotR = robot.botR.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();

        robot.botL.setTargetPosition(posBotL + 100);
        robot.topL.setTargetPosition(posTopL + 100);
        robot.botR.setTargetPosition(posBotR + 100);
        robot.topR.setTargetPosition(posTopR + 100);

        robot.botL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double power = -gamepad1.left_stick_y * 0.4;

        robot.botL.setPower(power);
        robot.topL.setPower(power);
        robot.botR.setPower(power);
        robot.topR.setPower(power);
    }

    void testAB(){
        if (testAB){
            int posBotL = robot.botL.getCurrentPosition();
            int posTopL = robot.topL.getCurrentPosition();
            int posBotR = robot.botR.getCurrentPosition();
            int posTopR = robot.topR.getCurrentPosition();

            double rotAmount = constants.CLICKS_PER_DEGREE * 360;
            double spinAmount = constants.CLICKS_PER_INCH * constants.WHEEL_CIRCUMFERENCE;

            robot.botL.setTargetPosition(posBotL + (int)(spinAmount + rotAmount));
            robot.topL.setTargetPosition(posTopL + (int)(-spinAmount + rotAmount));
            robot.botR.setTargetPosition(posBotR + (int)(spinAmount + rotAmount));
            robot.topR.setTargetPosition(posTopR + (int)(-spinAmount + rotAmount));
        }

        double power = 0.4;
        robot.botL.setPower(power);
        robot.topL.setPower(power);
        robot.botR.setPower(power);
        robot.topR.setPower(power);
    }

    public boolean noMovementRequests(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}