package org.firstinspires.ftc.teamcode.swerve.base;

import android.view.View;

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
import org.firstinspires.ftc.teamcode.common.kinematics.drive.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.SimplifiedKinematics;

@TeleOp(name="Revised BaseDrive", group="Drive")
//@Disabled
public class RevisedBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;

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
        kinematics = new RevisedKinematics(posSystem);
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        telemetry.addData("Turn Direction Left", kinematics.leftTurnDirectionW);
        telemetry.addData("Turn Direction Right", kinematics.rightTurnDirectionW);
        telemetry.addData("Spin Direction Left", kinematics.leftThrottle);
        telemetry.addData("Spin Direction Right", kinematics.rightThrottle);
        telemetry.addData("Turn Amount Left", kinematics.turnAmountL);
        telemetry.addData("Turn Amount Right", kinematics.turnAmountR);
        telemetry.addData("Target", kinematics.target);
        telemetry.addData("left w",  posSystem.getLeftWheelW());
        telemetry.addData("right w", posSystem.getRightWheelW());
        telemetry.addData("r", posSystem.getPositionArr()[4]);
        telemetry.addData("topl", robot.topL.getCurrentPosition());
        telemetry.addData("botl", robot.botL.getCurrentPosition());
        telemetry.addData("topr", robot.topR.getCurrentPosition());
        telemetry.addData("botr", robot.botR.getCurrentPosition());


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

        kinematics.logic(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y);

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