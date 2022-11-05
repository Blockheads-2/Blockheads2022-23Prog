package org.firstinspires.ftc.teamcode.teleop.test.arm;

import android.view.View;

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

    ElapsedTime resetTimer = new ElapsedTime();
    View relativeLayout;

    int prevPosition;

    private double power = 0.8;
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

        robot.armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void start(){
        prevPosition = robot.armBase.getCurrentPosition();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        telemetry.addData("Top", robot.armTop.getCurrentPosition());
        telemetry.addData("Bottom", robot.armBase.getCurrentPosition());
        telemetry.addData("Power", power);
        telemetry.addData("Height", armKinematics.findHeightToGround(armKinematics.getPsi(robot.armBase.getCurrentPosition()), armKinematics.getTheta(robot.armTop.getCurrentPosition())));

        telemetry.addData("Servo Position", robot.claw.getPosition());

        telemetry.update();
    }

    private void UpdateButton() {
        x.update(gamepad2.x);
        y.update(gamepad2.y);
        a.update(gamepad2.a);
        b.update(gamepad2.b);
    }

    void UpdatePlayer2(){
        setArmPower();
    }

    public void setTargetPositive(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

      //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armTop.setTargetPosition(topCurrent + 50);

     //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

      //  robot.armBase.setPower(0.3);
        robot.armTop.setPower(power);
    }

    public void setTargetNegative(){
        int baseDegree = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

       // robot.armBase.setTargetPosition(baseCurrent - 100);
        robot.armTop.setTargetPosition(topCurrent - 50);

      //  robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    //    robot.armBase.setPower(0.3);
        robot.armTop.setPower(power);
    }

    public void setTargetPositiveBase(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase.setTargetPosition(baseCurrent + 100);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase.setPower(power);
    }

    public void setTargetNegativeBase(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase.setTargetPosition(baseCurrent - 100);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase.setPower(power);
    }

    public void maintainHeightToGroundPositive(){
        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();


        robot.armBase.setTargetPosition((int)((baseCurrent + (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent - 10));

        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setPower(power);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void maintainHeightToGroundNegative(){
        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        robot.armBase.setTargetPosition((int)((baseCurrent - (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent + 10));

        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setPower(power);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void coneGrab(){
        robot.claw.setPosition(0.9);
    }

    public void coneBack(){
        robot.claw.setPosition(0.5);
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
        }
        if (gamepad2.x){
            setTargetNegative();
        }
        if(gamepad2.a){
            setTargetPositiveBase();
        }
        if(gamepad2.b){
            setTargetNegativeBase();
        }
        if (gamepad2.right_bumper){
            power += 0.1;
        }
        if (gamepad2.left_bumper){
            power -= 0.1;
        }
        if (gamepad2.dpad_right){
            coneGrab();
        }
        if (gamepad2.dpad_left){
            coneBack();
        }
        if (gamepad2.dpad_up){
           clawAngleUp();
        }
        if (gamepad2.dpad_down){
            clawAngleDown();
        }

        if (robot.armBase.getCurrentPosition() != prevPosition){
                robot.armBase.setTargetPosition(prevPosition);
                robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armBase.setPower(0.1);
        } else if (gamepad1.dpad_left){
            coneBack();
        } else if (robot.armBase.getCurrentPosition() != prevPosition){
            robot.armBase.setTargetPosition(prevPosition);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armBase.setPower(0.1);
        } else {
            robot.armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        prevPosition = robot.armBase.getCurrentPosition();

        if (power < 0) power = 0;
        else if (power > 1) power = 1;

        if(clawPosition < 0) clawPosition = 0;
        else if(clawPosition > 1) power = 1;
    }

    public void linearExtensionControl(){
        //angles for arm bottomg and top linkages
        double psi = armKinematics.getPsi(robot.armBase.getCurrentPosition());
        double theta = armKinematics.getTheta(robot.armTop.getCurrentPosition());

        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        int baseTarget = 0;
        int topTarget = 0;
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

        if (gamepad2.dpad_up){
            //high pos

        } else if (gamepad2.dpad_right){
            //mid pos

        } else if (gamepad2.dpad_down){
            //low pos

        } else if (gamepad2.dpad_left){
            //ground pos

        }
    }

    public void servoTester(){
        robot.claw.setPosition(0);
        robot.armServo.setPosition(0);
    }

    @Override
    public void stop() {
    }
}
