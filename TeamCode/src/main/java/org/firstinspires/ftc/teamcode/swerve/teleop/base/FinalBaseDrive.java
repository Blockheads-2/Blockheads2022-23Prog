package org.firstinspires.ftc.teamcode.swerve.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.teleop.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.pid.ArmPID;

@TeleOp(name="Final BaseDrive", group="Drive")
//@Disabled
public class FinalBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;

    Constants constants = new Constants();
    Reset reset;

    private enum TelemetryData{
        LEFT,
        RIGHT
    }
    TelemetryData tData = TelemetryData.LEFT;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    private double powerTopL = 0;
    private double powerBotL = 0;
    private double powerTopR = 0;
    private double powerBotR = 0;

    private int armTopPos = 0;
    private int armBotPos = 0;

    public int posTopArm = 0;

    //ARM ATTRIBUTES
    public int abPos = 0, atPos = 0;

    Button bottomButton = new Button();
    Button lowButton = new Button();
    Button midButton = new Button();
    Button highButton = new Button();
    Button testOne = new Button();
    Button testZero = new Button();
    Button testNegOne = new Button();
    Button zeroButton = new Button();

    Button clawAngleButton = new Button();
    Button clawGrabButton = new Button();
    Button a2 = new Button();
    Button b2 = new Button();

    boolean clawClose = false;
    boolean clawUp = false;
    public double clawAngle = 0;

    private int prevPosition = 0;

    ArmPID atPID = new ArmPID();
    ArmPID abrPID = new ArmPID();
    ArmPID ablPID = new ArmPID();

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
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        DriveTrainPowerEncoder();
        slowTurning();

        if (x.getState() == Button.State.TAP){
            tData = TelemetryData.LEFT;
        } else if (y.getState() == Button.State.TAP){
            tData = TelemetryData.RIGHT;
        }
//
//        if (a.getState() == Button.State.TAP){
//            kinematics.switchLeftSpinDirection();
//        } else if (b.getState() == Button.State.TAP){
//            kinematics.switchRightSpinDirection();
//        }

        if (a.getState() == Button.State.TAP){
            posSystem.resetOrientationIfOneEighty();
        }
        slowTurning();
    }

    void UpdatePlayer2(){
        ClawControl();
        ArmPresets();
    }

    void UpdateTelemetry(){
        telemetry.addData("Arm top pos", robot.at.getCurrentPosition());
        telemetry.addData("Arm bot pos", robot.abl.getCurrentPosition());
        telemetry.addData("Arm servo pos", robot.armServo.getPosition());

        telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        telemetry.addData("Left W",  posSystem.getLeftWheelW());
        telemetry.addData("Right W", posSystem.getRightWheelW());
        telemetry.addData("Optimized Left W", posSystem.optimizedCurrentWL);
        telemetry.addData("Optimized Right W", posSystem.optimizedCurrentWR);
//        telemetry.addData("Non left wheel Left W", PodL.nonRightStickCurrentW);
//        telemetry.addData("Non right wheel Right W", PodR.nonRightStickCurrentW);
//        telemetry.addData("R reference point", PodR.controlHeaderReference);

        telemetry.addData("R", posSystem.getPositionArr()[4]);

        telemetry.addData("Spin Direction (Left)", kinematics.leftThrottle);
        telemetry.addData("Spin Direction (Right)", kinematics.rightThrottle);

        telemetry.addData("Turn Amount (Left)", kinematics.turnAmountL);
        telemetry.addData("Turn Amount (Right)", kinematics.turnAmountR);

        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        telemetry.addData("Spin clicks target", kinematics.spinClicksL);
        telemetry.addData("Rotate clicks target",  kinematics.leftRotClicks);
        telemetry.addData("Spin clicks target", kinematics.spinClicksR);
        telemetry.addData("Rotate clicks target",  kinematics.rightRotClicks);

        telemetry.addData("Spin Power", kinematics.telSpinPower);
        telemetry.addData("Rotate Power L", kinematics.telLeftRotatePower);
        telemetry.addData("Rotate Power R", kinematics.telRightRotatePower);

        telemetry.addData("Drive Type", kinematics.getDriveType());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad2.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);

        bottomButton.update(gamepad2.dpad_down);
        lowButton.update(gamepad2.dpad_right);
        midButton.update(gamepad2.dpad_left);
        highButton.update(gamepad2.dpad_up);
        zeroButton.update(gamepad2.b);
    }

    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        kinematics.logic(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y); //wheelAllignment is one loop late.

        if (kinematics.getDriveType() == RevisedKinematics.DriveType.STOP){
            reset.reset(true);
            return;
        } else {
            reset.reset(false);
        }

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
        robot.topL.setPower(motorPower[0]);
        robot.botL.setPower(motorPower[1]);
        robot.topR.setPower(motorPower[2]);
        robot.botR.setPower(motorPower[3]);
    }

    public boolean noMovementRequests(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0);
    }

    void MoveArm(){
        if (gamepad2.y){
            armTopPos = armTopPos + 20;
        }
        if (gamepad2.a){
            armTopPos = armTopPos - 20;
        }
        if (gamepad2.b){
            armBotPos = armBotPos + 100;
        }
        if (gamepad2.x){
            armBotPos = armBotPos - 100;
        }
        if (gamepad2.right_bumper){
            armBotPos = armBotPos + 10;
        }
        if (gamepad2.right_bumper){
            armBotPos = armBotPos - 10;
        }
        if (armTopPos < 0){
            armTopPos = 0;
        }
        if (armBotPos < 0){
            armBotPos = 0;
        }

        robot.abl.setTargetPosition(armBotPos);
        robot.abr.setTargetPosition(armBotPos);
        robot.at.setTargetPosition(armTopPos);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(1);
        robot.abr.setPower(1);
        robot.at.setPower(1);
    }

    void ArmPresets(){
        if (bottomButton.is(Button.State.TAP)){
            clawAngle = constants.armServoBottom;
            atPID.setTargets(constants.topMotorBottom, robot.at.getCurrentPosition(), 0.4, 0, 0.2);
            ablPID.setTargets(constants.topMotorBottom, robot.abl.getCurrentPosition(), 0.4, 0, 0.2);
            abrPID.setTargets(constants.topMotorBottom, robot.abr.getCurrentPosition(), 0.4, 0, 0.2);

            robot.at.setTargetPosition(constants.topMotorBottom);
            robot.abl.setTargetPosition(constants.bottomMotorBottom);
            robot.abr.setTargetPosition(constants.bottomMotorBottom);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot.armServo.setPosition(constants.armServoBottom);

            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);



           /*
            robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

            */
        }

        if (lowButton.is(Button.State.TAP)){
            atPID.setTargets(constants.topMotorLow, robot.at.getCurrentPosition(), 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorLow, robot.abl.getCurrentPosition(), 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorLow, robot.abr.getCurrentPosition(), 0.4, 0, 0.2);

            robot.abl.setTargetPosition(constants.bottomMotorLow);
            robot.abr.setTargetPosition(constants.bottomMotorLow);
            robot.at.setTargetPosition(constants.topMotorLow);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            clawAngle = constants.armServoLow;
          //  robot.armServo.setPosition(constants.armServoLow);

           /* robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

            */
            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);

        }

        if (midButton.is(Button.State.TAP)){
            atPID.setTargets(constants.topMotorMid, robot.at.getCurrentPosition(), 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorMid, robot.abl.getCurrentPosition(), 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorMid, robot.abr.getCurrentPosition(), 0.4, 0, 0.2);

            robot.abl.setTargetPosition(constants.bottomMotorMid);
            robot.abr.setTargetPosition(constants.bottomMotorMid);
            robot.at.setTargetPosition(constants.topMotorMid);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            clawAngle = constants.armServoMid;
            //robot.armServo.setPosition(constants.armServoMid);

            /*robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

             */
            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);

        }

        if (highButton.is(Button.State.TAP)){
            atPID.setTargets(constants.topMotorHigh, robot.at.getCurrentPosition(), 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorHigh, robot.abl.getCurrentPosition(), 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorHigh, robot.abr.getCurrentPosition(), 0.4, 0, 0.2);

            robot.abl.setTargetPosition(constants.bottomMotorHigh);
            robot.abr.setTargetPosition(constants.bottomMotorHigh);
            robot.at.setTargetPosition(constants.topMotorHigh);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            clawAngle = constants.armServoHigh;
            //robot.armServo.setPosition(constants.armServoHigh);

            /*robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

             */
            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);

        }
/*
        if (zeroButton.is(Button.State.TAP)){
            robot.abl.setTargetPosition(0);
            robot.abr.setTargetPosition(0);
            robot.at.setTargetPosition(0);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armServo.setPosition(0);

            robot.abl.setPower(1);
            robot.abr.setPower(1);
            robot.at.setPower(1);
        }
 */
    }

    void ClawControl(){
        if (x.is(Button.State.TAP)){
            if (clawClose){
                robot.claw.setPosition(0.9);
            }
            else {
                robot.claw.setPosition(constants.openClaw);
            }
            clawClose = !clawClose;
        }


        clawAngle = clawAngle + (0.1 * gamepad2.right_trigger);
        clawAngle = clawAngle - (0.1 * gamepad2.left_trigger);
        if (clawAngle >= 0.7){
            clawAngle = 0.7;
        }
        if (clawAngle <= 0){
            clawAngle = 0;
        }


        robot.armServo.setPosition(clawAngle);
    }

    void slowTurning(){
        double turnPower = 0.5;
        if (gamepad1.left_trigger > 0){
            robot.topL.setPower(-gamepad1.left_trigger * turnPower);
            robot.botL.setPower(-gamepad1.left_trigger * turnPower);
            robot.topR.setPower(gamepad1.left_trigger * turnPower);
            robot.botR.setPower(gamepad1.left_trigger * turnPower);
        }
        if (gamepad1.right_trigger > 0){
            robot.topL.setPower(gamepad1.right_trigger * turnPower);
            robot.botL.setPower(gamepad1.right_trigger * turnPower);
            robot.topR.setPower(-gamepad1.right_trigger * turnPower);
            robot.botR.setPower(-gamepad1.right_trigger * turnPower);
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}