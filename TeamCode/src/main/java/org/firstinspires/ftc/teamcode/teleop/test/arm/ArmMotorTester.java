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

        robot.armBase1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.armBase1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void start(){
        prevPosition = robot.armBase1.getCurrentPosition();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        telemetry.addData("Top", robot.armTop.getCurrentPosition());
        telemetry.addData("Bottom", robot.armBase1.getCurrentPosition());
        telemetry.addData("Power", power);
        telemetry.addData("Height", armKinematics.findHeightToGround(armKinematics.getPsi(robot.armBase1.getCurrentPosition()), armKinematics.getTheta(robot.armTop.getCurrentPosition())));
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
        int baseCurrent = robot.armBase1.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

      //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armTop.setTargetPosition(topCurrent + 10);

     //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

      //  robot.armBase.setPower(0.3);
        robot.armTop.setPower(power);
    }

    public void setTargetNegative(){
        int baseDegree = robot.armBase1.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

       // robot.armBase.setTargetPosition(baseCurrent - 100);
        robot.armTop.setTargetPosition(topCurrent - 10);

      //  robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    //    robot.armBase.setPower(0.3);
        robot.armTop.setPower(power);
    }

    public void setTargetPositiveBase(){
        int baseCurrent = robot.armBase1.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase1.setTargetPosition(baseCurrent + 50);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase1.setPower(power);
    }

    public void setTargetNegativeBase(){
        int baseCurrent = robot.armBase1.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase1.setTargetPosition(baseCurrent - 50);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase1.setPower(power);
    }

    public void maintainHeightToGroundPositive(){
        double baseCurrent = robot.armBase1.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();


        robot.armBase1.setTargetPosition((int)((baseCurrent + (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent - 10));

        robot.armBase1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase1.setPower(power);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(power);
    }

    public void maintainHeightToGroundNegative(){
        double baseCurrent = robot.armBase1.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        robot.armBase1.setTargetPosition((int)((baseCurrent - (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent + 10));

        robot.armBase1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase1.setPower(power);

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

        if (robot.armBase1.getCurrentPosition() != prevPosition){
                robot.armBase1.setTargetPosition(prevPosition);
                robot.armBase1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armBase1.setPower(0.1);
        } else if (gamepad1.dpad_left){
            coneBack();
        }  else {
            robot.armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armBase1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        prevPosition = robot.armBase1.getCurrentPosition();

        if (power < 0) power = 0;
        else if (power > 1) power = 1;

        if(clawPosition < 0) clawPosition = 0;
        else if(clawPosition > 1) power = 1;
    }

    public void linearExtensionControl(){
        //angles for arm bottomg and top linkages
        double psi = armKinematics.getPsi(robot.armBase1.getCurrentPosition());
        double theta = armKinematics.getTheta(robot.armTop.getCurrentPosition());
        double omega = armKinematics.getOmega(robot.armServo.getPosition());

        //current click position
        double baseCurrent = robot.armBase1.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();
        double armServoCurrent = robot.armServo.getPosition();

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
