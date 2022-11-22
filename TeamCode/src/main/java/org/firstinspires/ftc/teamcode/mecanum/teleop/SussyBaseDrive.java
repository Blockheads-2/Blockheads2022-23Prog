package org.firstinspires.ftc.teamcode.mecanum.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanum.common.MecanumConstants;
import org.firstinspires.ftc.teamcode.mecanum.common.SussyHardwareDrive;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Button;

@TeleOp(name="Sussy Base Drive", group="Drive")
//@Disabled
public class SussyBaseDrive extends OpMode{
    /* Declare OpMode members. */
    SussyHardwareDrive robot = new SussyHardwareDrive();
    MecanumConstants constants = new MecanumConstants();
    private ElapsedTime runtime = new ElapsedTime();

    Button bottomButton = new Button();
    Button lowButton = new Button();
    Button midButton = new Button();
    Button highButton = new Button();
    Button testOne = new Button();
    Button testZero = new Button();
    Button testNegOne = new Button();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);



        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

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

    void UpdateButton(){
        bottomButton.update(gamepad2.x);
        lowButton.update(gamepad2.a);
        midButton.update(gamepad2.b);
        highButton.update(gamepad2.y);
        testOne.update(gamepad1.y);
        testZero.update(gamepad1.b);
        testNegOne.update(gamepad1.a);

        bottomButton.update(gamepad2.dpad_down);
        lowButton.update(gamepad2.dpad_left);
        midButton.update(gamepad2.dpad_right);
        highButton.update(gamepad2.dpad_up);
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
        //OscillateServo();
    }

    void UpdateTelemetry(){

        telemetry.addData("LF", robot.lf.getCurrentPosition());
        telemetry.addData("LB", robot.lb.getCurrentPosition());
        telemetry.addData("RF", robot.rf.getCurrentPosition());
        telemetry.addData("RB", robot.rb.getCurrentPosition());

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

    void armMovement(){
        //abl, abr, at

        if (bottomButton.is(Button.State.TAP)){
            robot.abl.setTargetPosition(constants.bottomMotorBottom);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);
            //robot.abr.setTargetPosition(constants.bottomMotorBottom);
            //robot.at.setTargetPosition(constants.topMotorBottom);
        }

        if (lowButton.is(Button.State.TAP)){
            robot.abl.setTargetPosition(constants.bottomMotorLow);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            //robot.abr.setTargetPosition(constants.bottomMotorLow);
            //robot.at.setTargetPosition(constants.topMotorLow);
        }

        if (midButton.is(Button.State.TAP)){
            robot.abl.setTargetPosition(constants.bottomMotorMid);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            //robot.abr.setTargetPosition(constants.bottomMotorMid);
            //robot.at.setTargetPosition(constants.topMotorMid);
        }

        if (highButton.is(Button.State.TAP)){
            robot.abl.setTargetPosition(constants.bottomMotorHigh);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.5);

            //robot.abr.setTargetPosition(constants.bottomMotorHigh);
            //robot.at.setTargetPosition(constants.topMotorHigh);
        }

    }

    void testServos(){
        if (testOne.is(Button.State.TAP)){
            robot.armServo.setPosition(1);
            robot.claw.setPosition(1);
        }
        if (testZero.is(Button.State.TAP)){
            robot.armServo.setPosition(0);
            robot.claw.setPosition(0);
        }
        if (testNegOne.is(Button.State.TAP)){
            robot.armServo.setPosition(-1);
            robot.claw.setPosition(-1);
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
