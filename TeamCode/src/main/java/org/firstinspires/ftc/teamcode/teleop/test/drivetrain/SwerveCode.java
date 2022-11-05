package org.firstinspires.ftc.teamcode.teleop.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Swerve Code", group="Drive")
//@Disabled
public class SwerveCode extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Kinematics kinematics;
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();

    ElapsedTime accelerationTimer = new ElapsedTime();
    boolean isAccelerateCycle = false;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    private int rotateR;
    private int rotateL;

    public enum State{
        DRIVE,
        RESET
    }
    State driveState = State.DRIVE;
    Reset reset;

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
        reset = new Reset(robot,posSystem);

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
       // DriveTrainBasePower();
        if (driveState == State.DRIVE){
            DriveTrainPowerEncoder();
            reset.reset(false);
        } else if (driveState == State.RESET){
            reset.reset(true);
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

//        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
//        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        telemetry.addData("rotateR target", rotateR);
        telemetry.addData("rotateL target", rotateL);
        telemetry.addData("isBusy", robot.wheelsAreBusy());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);

        if (x.getState() == Button.State.TAP){
            driveState = State.RESET;

        } else if (y.getState() == Button.State.TAP){
            driveState = State.DRIVE;
        }
    }


    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();
        int posBotR = robot.botR.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();

        double alpha = 0.5;
        double beta = 1 - alpha;

        int distanceTopL = (int) (gamepad1.left_stick_y * 100 * beta);
        int distanceBotL = (int) (-gamepad1.left_stick_y * 100 * beta);
        int distanceTopR = distanceTopL;
        int distanceBotR = distanceBotL;

        int rotationalTopL = -(int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalBotL = -(int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalTopR = -rotationalTopL;
        int rotationalBotR = -rotationalBotL;

        robot.botL.setTargetPosition(posBotL + distanceBotL + rotationalBotL);
        robot.topL.setTargetPosition(posTopL + distanceTopL + rotationalTopL);
        robot.botR.setTargetPosition(posBotR + distanceBotR + rotationalBotR);
        robot.topR.setTargetPosition(posTopR + distanceTopR + rotationalTopR);

        robot.botL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.botL.setPower(accelerator(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha) * 0.3);
        robot.topL.setPower(accelerator(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha) * 0.3);
        robot.botR.setPower(accelerator(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha) * 0.3);
        robot.topR.setPower(accelerator(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha) * 0.3);
    }

    public double accelerator(double power){
        if (power == 0) return 0.0;

        if (!isAccelerateCycle){
            accelerationTimer.reset();
            isAccelerateCycle = true;
        }
        double accelerationFactor = (Math.tanh(accelerationTimer.milliseconds() - 1.5) / 2.5) + 0.6;
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
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