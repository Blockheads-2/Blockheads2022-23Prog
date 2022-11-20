package org.firstinspires.ftc.teamcode.swerve.test.arm;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.kinematics.arm.ArmKinematics;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;


@TeleOp(name = "Arm Test Move", group = "Drive")

public class ArmMotorTester extends OpMode{
    //local class objects
    HardwareDrive robot = new HardwareDrive();
    ArmKinematics armKinematics = new ArmKinematics();
    Constants constants = new Constants();

    //buttons
    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();
    Button dUp = new Button();
    Button dRight = new Button();
    Button dDown = new Button();
    Button dLeft = new Button();

    ElapsedTime resetTimer = new ElapsedTime();
    View relativeLayout;

    int prevPosition = robot.armBaseLeft.getCurrentPosition();

    private double power = 0.5;
    private double clawPosition;

    @Override
    public void init(){
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        //  robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawPosition = robot.armServo.getPosition();

        robot.armBaseLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armBaseRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armBaseLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armBaseRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start(){
        prevPosition = robot.armBaseLeft.getCurrentPosition();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        telemetry.addData("Top", robot.armTop.getCurrentPosition());
        telemetry.addData("Bottom", robot.armBaseLeft.getCurrentPosition());
        telemetry.addData("Power", power);
        telemetry.addData("Height", armKinematics.findHeightToGround(armKinematics.getPsi(robot.armBaseLeft.getCurrentPosition()), armKinematics.getTheta(robot.armTop.getCurrentPosition())));
        telemetry.update();
    }

    private void UpdateButton() {
        x.update(gamepad2.x);
        y.update(gamepad2.y);
        a.update(gamepad2.a);
        b.update(gamepad2.b);
        dUp.update(gamepad2.dpad_up);
        dRight.update(gamepad2.dpad_right);
        dDown.update(gamepad2.dpad_down);
        dLeft.update(gamepad2.dpad_left);
    }

    void UpdatePlayer2(){
        setArmPower();
    }

    public void setTargetPositive(){
        int topCurrent = robot.armTop.getCurrentPosition();

        robot.armTop.setTargetPosition(topCurrent + 10);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void setTargetNegative(){
        int topCurrent = robot.armTop.getCurrentPosition();

        robot.armTop.setTargetPosition(topCurrent - 10);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void setTargetPositiveBase(){
        int baseLeftCurrent = robot.armBaseLeft.getCurrentPosition();
        int baseRightCurrent = robot.armBaseRight.getCurrentPosition();

        robot.armBaseLeft.setTargetPosition(baseLeftCurrent + 100);
        robot.armBaseRight.setTargetPosition(baseRightCurrent + 100);

        robot.armBaseLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBaseRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.armBaseLeft.setPower(power);
        robot.armBaseRight.setPower(power);
    }

    public void setTargetNegativeBase(){
        int baseLeftCurrent = robot.armBaseLeft.getCurrentPosition();
        int baseRightCurrent = robot.armBaseRight.getCurrentPosition();

        robot.armBaseLeft.setTargetPosition(baseLeftCurrent - 100);
        robot.armBaseRight.setTargetPosition(baseRightCurrent - 100);

        robot.armBaseLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBaseRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.armBaseLeft.setPower(power);
        robot.armBaseRight.setPower(power);
    }

    public void maintainHeightToGroundPositive(){
        int baseLeftCurrent = robot.armBaseLeft.getCurrentPosition();
        int baseRightCurrent = robot.armBaseRight.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        robot.armBaseLeft.setTargetPosition((int)((baseLeftCurrent + (10 * constants.RATIO_CLICKS))));
        robot.armBaseRight.setTargetPosition((int)((baseRightCurrent + (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition(topCurrent - 10);

        robot.armBaseLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBaseLeft.setPower(power);

        robot.armBaseRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armBaseRight.setPower(power);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void maintainHeightToGroundNegative(){
        double baseCurrent = robot.armBaseLeft.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        robot.armBaseLeft.setTargetPosition((int)((baseCurrent - (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent + 10));

        robot.armBaseLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBaseLeft.setPower(power);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void coneGrab(){
        robot.claw.setPosition(0.85);
    }

    public void coneBack(){
        robot.claw.setPosition(0.2);
    }

    public void clawAngleUp(){
        clawPosition += 0.1;
        robot.armServo.setPosition(clawPosition);
    }

    public void clawAngleDown(){
        clawPosition -= 0.1;
        robot.armServo.setPosition(clawPosition);
    }

    void setArmPower(){
        if (gamepad2.y){
            setTargetPositive();
        } else if (gamepad2.x){
            setTargetNegative();
        } else if(gamepad2.a){
            setTargetPositiveBase();
        } else if(gamepad2.b){
            setTargetNegativeBase();
        } else if (gamepad2.right_bumper){
            power += 0.1;
        } else if (gamepad2.left_bumper){
            power -= 0.1;
        } else if (gamepad2.dpad_right){
            coneGrab();
        } else if (gamepad2.dpad_left){
            coneBack();
        } else if (gamepad2.dpad_up){
            clawAngleUp();
        } else if (gamepad2.dpad_down){
            clawAngleDown();
        }

        if (robot.armBaseLeft.getCurrentPosition() != prevPosition){
                robot.armBaseLeft.setTargetPosition(prevPosition);
                robot.armBaseLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armBaseLeft.setPower(0.1);
        } else {
            robot.armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armBaseLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        prevPosition = robot.armBaseLeft.getCurrentPosition();

        if (power < 0) power = 0;
        else if (power > 1) power = 1;

        if(clawPosition < 0) clawPosition = 0;
        else if(clawPosition > 1) power = 1;
    }

    public void positions(){
        //positions: ground, low, mid, high
        //clicks:
            //ground(base, top) ()
            //low(base, top) ()
            //mid(base, top) ()
            //high(base, top) ()

        //button
            //dpad up: high
            //dpad right: mid
            //dpad down: low
            //dpad left: ground

        if (dUp.getState().equals(Button.State.TAP)){
            //high pos

        } else if (dRight.getState().equals(Button.State.TAP)){
            //mid pos

        } else if (dDown.getState().equals(Button.State.TAP)){
            //low pos

        } else if (dLeft.getState().equals(Button.State.TAP)){
            //ground pos

        }
    }

    @Override
    public void stop() {
    }
}
