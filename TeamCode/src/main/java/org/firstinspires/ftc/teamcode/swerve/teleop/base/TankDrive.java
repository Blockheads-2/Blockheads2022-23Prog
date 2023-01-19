package org.firstinspires.ftc.teamcode.swerve.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Tank Drive", group="Drive")
@Disabled
public class TankDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Kinematics kinematics;

    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();

    Accelerator acceleratorR = new Accelerator();
    Accelerator acceleratorL = new Accelerator();
    Reset reset;


    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    private int targetTopR;
    private int targetBotR;
    private int targetTopL;
    private int targetBotL;
    private double powerL;
    private double powerR;

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        kinematics = new Kinematics(posSystem);
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

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
        if (noMovementRequests()){
            reset.reset(true);
        } else {
            reset.reset(false);
            DriveTrainPowerEncoder();
        }
    }

    void UpdatePlayer2(){

    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);
        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());

        double[] posData = posSystem.getPositionArr();

        telemetry.addData("Xpos", posData[0]);
        telemetry.addData("Ypos", posData[1]);
        telemetry.addData("Left W", posData[2]);
        telemetry.addData("Right W", posData[3]);
        telemetry.addData("R", posData[4]);

        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());
        telemetry.addData("PowerR", powerR);
        telemetry.addData("PowerL", powerL);

//        telemetry.addData("target topL", targetTopL);
//        telemetry.addData("target botL", targetBotL);
//        telemetry.addData("target topR", targetTopR);
//        telemetry.addData("target botR", targetBotR);

        telemetry.addData("isBusy", robot.wheelsAreBusy());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }


    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition() ;
        int posBotR = robot.botR.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();

        int distanceTopL = (int)(-gamepad1.right_stick_y * 100);
        int distanceBotL = (int)(gamepad1.right_stick_y * 100);
        int distanceTopR = (int)(-gamepad1.left_stick_y * 100);
        int distanceBotR = (int)(gamepad1.left_stick_y * 100);

        powerL = acceleratorL.update(gamepad1.right_stick_y) * constants.POWER_LIMITER;
        powerR = acceleratorR.update(gamepad1.left_stick_y) * constants.POWER_LIMITER;

        robot.botL.setTargetPosition(posBotL + distanceBotL);
        robot.topL.setTargetPosition(posTopL + distanceTopL);
        robot.botR.setTargetPosition(posBotR + distanceBotR);
        robot.topR.setTargetPosition(posTopR + distanceTopR);

        if (powerL != 0 && powerR != 0) {
            robot.botL.setPower(-powerL);
            robot.topL.setPower(powerL);
            robot.botR.setPower(powerR);
            robot.topR.setPower(-powerR);

        } else if (powerL == 0){ //powerR
//            powerL = powerR / 2.0;
            robot.botL.setPower(0);
            robot.topL.setPower(0);
            robot.botR.setPower(powerR);
            robot.topR.setPower(-powerR);

        } else { //powerL
            robot.botL.setPower(-powerL);
            robot.topL.setPower(powerL);
            robot.botR.setPower(0);
            robot.topR.setPower(0);
        }

        if (powerL == 0 && powerR != 0){
            powerL = powerR / 2.0;

            robot.botL.setTargetPosition(posBotL + (distanceBotL/2));
            robot.topL.setTargetPosition(posTopL + (distanceTopL/2));
        } else if (powerR == 0 && powerL != 0){ //powerR == 0
            powerR = powerL / 2.0;

            robot.botR.setTargetPosition(posBotR + (distanceBotR/2));
            robot.topR.setTargetPosition(posTopR + (distanceTopR/2));
        }

        robot.botL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.botL.setPower(powerL);
        robot.topL.setPower(powerL);
        robot.botR.setPower(powerR);
        robot.topR.setPower(powerR);
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