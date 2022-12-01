package org.firstinspires.ftc.teamcode.mecanum.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanum.common.Constants;
import org.firstinspires.ftc.teamcode.mecanum.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Button;

@TeleOp(name="Sussy Base Drive", group="Drive")
//@Disabled
public class SussyBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();

    public int abPos = 0, atPos = 0;

    Button bottomButton = new Button();
    Button lowButton = new Button();
    Button midButton = new Button();
    Button highButton = new Button();
    Button testOne = new Button();
    Button testZero = new Button();
    Button testNegOne = new Button();

    Button y = new Button();
    Button x = new Button();
    Button a = new Button();
    Button b = new Button();

    private int prevPosition = 0;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        prevPosition = robot.abl.getCurrentPosition();

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateTelemetry();
        UpdateButton();
    }

    void updateTelemetry(){
        telemetry.addData("Top", robot.at.getCurrentPosition());
        telemetry.addData("Base Right", robot.abr.getCurrentPosition());
        telemetry.addData("Base Left", robot.abl.getCurrentPosition());
        telemetry.update();
    }

    void UpdateButton(){

        testOne.update(gamepad1.a);
        testZero.update(gamepad1.b);
        testNegOne.update(gamepad1.y);

        bottomButton.update(gamepad2.dpad_down);
        lowButton.update(gamepad2.dpad_left);
        midButton.update(gamepad2.dpad_right);
        highButton.update(gamepad2.dpad_up);

        y.update(gamepad2.y);
        x.update(gamepad2.x);
        a.update(gamepad2.a);
        b.update(gamepad2.b);
    }

    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        DriveMicroAdjust(0.4);
        testServos();
        //OscillateServo();
    }

    void UpdatePlayer2(){
        armMovement();
        setArmPower();
        //OscillateServo();
    }

    void UpdateTelemetry(){

        telemetry.addData("LF", robot.lf.getCurrentPosition());
        telemetry.addData("LB", robot.lb.getCurrentPosition());
        telemetry.addData("RF", robot.rf.getCurrentPosition());
        telemetry.addData("RB", robot.rb.getCurrentPosition());

        telemetry.addData("Current Position", robot.at.getCurrentPosition());
        telemetry.addData("Current Bottom Arm # Clicks", robot.abl.getCurrentPosition());

        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn


        robot.lf.setPower((directionY + directionR + directionX) * drivePower);
        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower);
        robot.rb.setPower((directionY - directionR + directionX) * drivePower);

    }

    void DriveMicroAdjust(double power){
        if (gamepad1.dpad_up){
            robot.lf.setPower(power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_down){
            robot.lf.setPower(-power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(-power);
        }
        else if (gamepad1.dpad_right){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_left){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }

        if (gamepad1.left_trigger == 1){
            robot.lf.setPower(-power);
            robot.rf.setPower(power);
            robot.lb.setPower(-power);
            robot.rb.setPower(power);
        }
        else if (gamepad1.right_trigger == 1){
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
        }
    }

    public void setTargetPositive(){
        int topCurrent = robot.at.getCurrentPosition();

        telemetry.addData("Goal Position", topCurrent + 10);

        robot.at.setTargetPosition(topCurrent + 30);
        robot.at.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.at.setPower(0.75);
    }

    public void setTargetNegative(){
        int topCurrent = robot.at.getCurrentPosition();

        robot.at.setTargetPosition(topCurrent - 30);
        robot.at.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.at.setPower(0.75);
    }

    public void setTargetPositiveBase(){
        int baseLeftCurrent = robot.abl.getCurrentPosition();
        int baseRightCurrent = robot.abr.getCurrentPosition();

        robot.abl.setTargetPosition(baseLeftCurrent + 20);
        robot.abr.setTargetPosition(baseRightCurrent + 20);

        robot.abl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(0.3);
        robot.abr.setPower(0.3);
    }

    public void setTargetNegativeBase(){
        int baseLeftCurrent = robot.abl.getCurrentPosition();
        int baseRightCurrent = robot.abr.getCurrentPosition();

        robot.abl.setTargetPosition(baseLeftCurrent - 20);
        robot.abr.setTargetPosition(baseRightCurrent - 20);

        robot.abl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(0.3);
        robot.abr.setPower(0.3);
    }

    void setArmPower(){
        if (gamepad2.y){
            setTargetPositive();
        } else if (gamepad2.x){
            setTargetNegative();
        } else if(a.is(Button.State.TAP)){
            setTargetPositiveBase();
        } else if(b.is(Button.State.TAP)){
            setTargetNegativeBase();
        }

        if (robot.abl.getCurrentPosition() != prevPosition){
            robot.abl.setTargetPosition(prevPosition);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.1);
            robot.abr.setPower(0.1);
        } else {
            robot.at.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.abl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.abr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        prevPosition = robot.abl.getCurrentPosition();
    }

    void armMovement(){
        //abl, abr, at

        if (bottomButton.is(Button.State.TAP)){
            atPos -= 10;
            robot.at.setTargetPosition(atPos);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);


            /*robot.abl.setTargetPosition(constants.bottomMotorBottom);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(constants.bottomMotorBottom);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);

            robot.at.setTargetPosition(constants.topMotorBottom);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);*/
        }

        if (lowButton.is(Button.State.TAP)){

            if (abPos >= 10)
                abPos -= 10;

            robot.abl.setTargetPosition(abPos);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(abPos);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);
            telemetry.addData("Current Top Arm # Clicks", abPos);

            /*robot.abl.setTargetPosition(constants.bottomMotorLow);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(constants.bottomMotorLow);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);

            robot.at.setTargetPosition(constants.topMotorLow);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);*/
        }

        if (midButton.is(Button.State.TAP)){
            abPos += 10;
            robot.abl.setTargetPosition(abPos);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(abPos);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);

            telemetry.addData("Current Top Arm # Clicks", abPos);

            /*robot.abl.setTargetPosition(constants.bottomMotorMid);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(constants.bottomMotorMid);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);

            robot.at.setTargetPosition(constants.topMotorMid);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);*/
        }

        if (highButton.is(Button.State.TAP)){
            atPos += 10;

            robot.at.setTargetPosition(atPos);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);
            telemetry.addData("Current Bottom Arm # Clicks", atPos);



            /*robot.abl.setTargetPosition(constants.bottomMotorHigh);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            robot.abr.setTargetPosition(constants.bottomMotorHigh);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setPower(0.5);

            robot.at.setTargetPosition(constants.topMotorHigh);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower(0.5);*/
        }
    }

    void testServos(){
        if (testOne.is(Button.State.TAP)){
            robot.armServo.setPosition(constants.clawDown);
            robot.claw.setPosition(constants.closeClaw);
        }
        if (testZero.is(Button.State.TAP)) {
            robot.armServo.setPosition(constants.clawUp);
            robot.claw.setPosition(constants.openClaw);
        }
    }

    double DriveTrainSpeed(){
        double drivePower = 0.75;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;

        return drivePower;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
