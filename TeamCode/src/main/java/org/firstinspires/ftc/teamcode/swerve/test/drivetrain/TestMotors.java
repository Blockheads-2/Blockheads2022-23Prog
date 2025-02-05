package org.firstinspires.ftc.teamcode.swerve.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.Button;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.swerve.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;

@TeleOp(name="Test Motors", group="Drive")
//@Disabled
public class TestMotors extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    RevisedKinematics kinematics;
    GlobalPosSystem posSystem;
    Constants constants = new Constants();
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();
    int distanceClicks;
    int rotClicks;

    double avgClicksPerLoop = 0;
    double avgClicksPerSecond = 0;
    double prevClicks = 0;
    int loopCounter = 0;
    ElapsedTime avgCliskPerLoopTimer = new ElapsedTime();
    double prevTime = 0;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

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
//        kinematics = new RevisedKinematics(posSystem);
        posSystem.grabKinematics(kinematics);
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        double distance = constants.WHEEL_CIRCUMFERENCE;
        double rot = 90;
//        distanceClicks = (int)(distance * constants.CLICKS_PER_INCH); //rotation clicks
//        rotClicks = (int)(rot * constants.CLICKS_PER_DEGREE);
        distanceClicks=  (int)(distance * constants.CLICKS_PER_INCH);
        rotClicks = (int)(rot * constants.CLICKS_PER_DEGREE);
//        drive(distanceClicks, rotClicks);


        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        //  robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed
        avgCliskPerLoopTimer.reset();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        DriveTrainPowerEncoder();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        telemetry.addData("Xpos", posSystem.getPositionArr()[0]);
        telemetry.addData("Ypos", posSystem.getPositionArr()[1]);
        telemetry.addData("left W", posSystem.getPositionArr()[2]);
        telemetry.addData("right W", posSystem.getPositionArr()[3]);
        telemetry.addData("R", posSystem.getPositionArr()[4]);
        telemetry.addData("Target Rot", rotClicks);
        telemetry.addData("Target Distance", distanceClicks);

        telemetry.addData("TopL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("BotL Clicks", robot.botL.getCurrentPosition());
        telemetry.addData("TopR Clicks", robot.topR.getCurrentPosition());
        telemetry.addData("BotR Clicks", robot.botR.getCurrentPosition());

        telemetry.addData("TopL Mode", robot.topL.getMode());
        telemetry.addData("BotL Mode", robot.botL.getMode());
        telemetry.addData("TopR Mode", robot.topR.getMode());
        telemetry.addData("BotR Mode", robot.botR.getMode());

        telemetry.addData("Average Clicks per Loop", avgClicksPerLoop);
        telemetry.addData("Average Clicks per Second", avgClicksPerSecond);


        telemetry.update();
    }


    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }


    void DriveTrainPowerEncoder(){
        loopCounter++;

        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - constants.SPIN_CLICK_FACTOR);
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + constants.SPIN_CLICK_FACTOR);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - constants.SPIN_CLICK_FACTOR);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + constants.SPIN_CLICK_FACTOR);

        posSystem.calculatePos();

        robot.botL.setPower(1);
        robot.topL.setPower(1);
        robot.botR.setPower(1);
        robot.topR.setPower(1);

        avgClicksPerLoop = robot.topL.getCurrentPosition() / (double)loopCounter;
        double deltaTime = (avgCliskPerLoopTimer.milliseconds() - prevTime) / 1000.0;
        double deltaClicks = robot.topL.getCurrentPosition() / (double)prevClicks;
        avgClicksPerSecond = deltaClicks / deltaTime;

        prevTime = avgCliskPerLoopTimer.milliseconds();
        prevClicks = robot.topL.getCurrentPosition();
    }

    public void drive(int distanceClicks, int rotClicks) {
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - distanceClicks + rotClicks);
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceClicks + rotClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - distanceClicks + rotClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + distanceClicks + rotClicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;
        if (degrees == -180) degrees = 180;

        if (degrees < -180){
            degrees = 180 - (Math.abs(degrees) - 180);
        } else if (degrees > 180){
            degrees = -180 + (Math.abs(degrees) - 180);
        }
        return degrees;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}