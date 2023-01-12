package org.firstinspires.ftc.teamcode.swerve.teleop.base;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.SwervePod;
import org.firstinspires.ftc.teamcode.common.pid.ArmPID;

@TeleOp(name="Final BaseDrive", group="Drive")
//@Disabled
public class FinalBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Thread gpsUpdateThread;
    RevisedKinematics kinematics;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    Constants constants = new Constants();
    Reset reset;

    SwervePod PodR = new SwervePod(constants.initDirectionRight, SwervePod.Side.RIGHT);
    SwervePod PodL = new SwervePod(constants.initDirectionLeft, SwervePod.Side.LEFT);

    private enum TelemetryData{
        LEFT,
        RIGHT
    }
    TelemetryData tData = TelemetryData.LEFT;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    int[] targetClicks = new int[4];
    double[] motorPower = new double[4];

    //ARM ATTRIBUTES
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
    Button leftBumpy = new Button();
    Button rightBumpy = new Button();

    boolean clawClose = false;
    boolean clawUp = false;
    public double clawAngle = 0;
    boolean lowerArmCycle = false;
    boolean lowerAllTheWay = false;

    int stackClawPos = 1;

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
        kinematics = new RevisedKinematics(posSystem, PodL, PodR);
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");

//        robot.abl.setTargetPosition(constants.INIT_ARMBASE_POS);
//        robot.abr.setTargetPosition(constants.INIT_ARMBASE_POS);
//        robot.at.setTargetPosition(0);
//
//        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        gpsUpdateThread = new Thread(posSystem);
//        gpsUpdateThread.start();
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"

//        robot.abl.setPower(0.7);
//        robot.abr.setPower(0.7);
//        robot.at.setPower(0.7);

    }

    @Override
    public void start() { //When "start" is pressed
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        robot.abl.setTargetPosition(robot.abl.getCurrentPosition());
//        robot.abr.setTargetPosition(robot.abr.getCurrentPosition());
//        robot.abr.setTargetPosition(robot.at.getCurrentPosition());
//        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        robot.abl.setPower(0.7);
//        robot.abr.setPower(0.7);
//        robot.at.setPower(0.7);
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
        ClawControl();
        ArmPresets();
        UltraMegaArmPresets();
    }

    void UpdateTelemetry(){
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("IsAlligned", posSystem.isAlligned());
        telemetry.addData("Eligible for turning", posSystem.eligibleForTurning());
        telemetry.addData("Trying to turn but can't", kinematics.tryingToTurnButCant);
        telemetry.addData("First movement", kinematics.firstMovement);
        telemetry.addData("PodL initpole", PodL.initPole);

        telemetry.addData("PodR initPole", PodR.initPole);

        telemetry.addData("Leftstick X", gamepad1.left_stick_x);
        telemetry.addData("Leftstick Y", -gamepad1.left_stick_y);
        telemetry.addData("Rightstick X", gamepad1.right_stick_x);
        telemetry.addData("Rightstick Y", -gamepad1.right_stick_y);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("left trigger", -gamepad1.left_trigger);

        telemetry.addData("Arm top pos", robot.at.getCurrentPosition());
        telemetry.addData("Arm bot pos", robot.abl.getCurrentPosition());
        telemetry.addData("Arm servo pos", robot.armServo.getPosition());

        telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        telemetry.addData("Left W",  posSystem.getLeftWheelW());
        telemetry.addData("Right W", posSystem.getRightWheelW());
        telemetry.addData("Optimized Left W", PodL.optimizedCurrentW);
        telemetry.addData("Optimized Right W", PodR.optimizedCurrentW);
        telemetry.addData("R", posSystem.getPositionArr()[4]);

        telemetry.addData("Spin Direction (Left)", PodL.output.get("direction"));
        telemetry.addData("Spin Direction (Right)", PodR.output.get("direction"));
//
        telemetry.addData("target", kinematics.target);
        telemetry.addData("Turn Amount (Left)", PodL.getTurnAmount());
        telemetry.addData("Turn Amount (Right)", PodR.getTurnAmount());
        telemetry.addData("Throttle (Left)", PodL.output.get("throttle"));
        telemetry.addData("Throttle (Right)", PodR.output.get("throttle"));

        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        telemetry.addData("Left Spin Clicks Target", PodL.output.get("spinClicksTarget"));
        telemetry.addData("Left Rotate Clicks target",  PodL.output.get("rotClicksTarget"));
        telemetry.addData("Right Spin clicks target", PodR.output.get("spinClicksTarget"));
        telemetry.addData("Right Rotate clicks target",  PodR.output.get("rotClicksTarget"));
        telemetry.addData("topL velocity", robot.topL.getVelocity()); //ticks per second
        telemetry.addData("botL velocity", robot.botL.getVelocity()); //ticks per second
        telemetry.addData("topR velocity", robot.topR.getVelocity());
        telemetry.addData("botR velocity", robot.botR.getVelocity());


        telemetry.addData("Power TopL", PodL.output.get("power"));
        telemetry.addData("Power BotL", PodL.output.get("power"));
        telemetry.addData("Power TopR", PodR.output.get("power"));
        telemetry.addData("Power BotR", PodR.output.get("power"));
        telemetry.addData("Clicks Target TopL", robot.topL.getTargetPosition() - robot.topL.getCurrentPosition());
        telemetry.addData("Clicks Target BotL",robot.botL.getTargetPosition() - robot.botL.getCurrentPosition());
        telemetry.addData("Clicks Target TopR", robot.topR.getTargetPosition() - robot.topR.getCurrentPosition());
        telemetry.addData("Clicks Target BotR",robot.botR.getTargetPosition() - robot.botR.getCurrentPosition());

        telemetry.addData("Drive Type", kinematics.getDriveType());
        telemetry.addData("resetCycle?", kinematics.resestCycle);


        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);

        bottomButton.update(gamepad2.dpad_down);
        lowButton.update(gamepad2.dpad_right);
        midButton.update(gamepad2.dpad_left);
        highButton.update(gamepad2.dpad_up);
        zeroButton.update(gamepad2.b);
        leftBumpy.update(gamepad2.left_bumper);
        rightBumpy.update(gamepad2.right_bumper);
    }

    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();
        kinematics.logic(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.right_trigger, -gamepad1.left_trigger); //wheelAllignment is one loop late.

//        if (kinematics.getDriveType() == RevisedKinematics.DriveType.STOP){
//            boolean wheelsAreAlligned = posSystem.isAlligned();
//            boolean eligibleForTurning = posSystem.eligibleForTurning();
//            if (!wheelsAreAlligned || (!eligibleForTurning && kinematics.tryingToTurnOrSpline)){
//                reset.reset(true);
//                telemetry.addData("Reset", true);
//            }
//            return;
//        } else {
//            reset.reset(false);
//            telemetry.addData("Reset", false);
//        }

        targetClicks = kinematics.getClicks();
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

        robot.topL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motorPower = kinematics.getPower();
        robot.topL.setPower(motorPower[0]);
        robot.botL.setPower(motorPower[1]);
        robot.topR.setPower(motorPower[2]);
        robot.botR.setPower(motorPower[3]);
//        robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
//        robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
//        robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
//        robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);
    }

    void ArmPresets(){
        if (bottomButton.is(Button.State.TAP)){
            lowerArmCycle = true;
            lowerAllTheWay = true;
        }

        if (lowButton.is(Button.State.TAP)){
            lowerArmCycle = true;
        }

        lowerArm();
        lowerAllTheWay();

        if (midButton.is(Button.State.TAP)){
            atPID.setTargets(constants.topMotorMid, 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorMid, 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorMid, 0.4, 0, 0.2);

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
            atPID.setTargets(constants.topMotorHigh, 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorHigh, 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorHigh, 0.4, 0, 0.2);

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
    }

    void UltraMegaArmPresets(){
        if (leftBumpy.is(Button.State.TAP)){
             stackClawPos--;
        }
        if (rightBumpy.is(Button.State.TAP)){
            stackClawPos++;
        }
        if (stackClawPos<1){
            stackClawPos = 1;
        }
        switch (stackClawPos) {
            case 1: {
                setArmPos(Constants.topMotor1, Constants.bottomMotor1);
            }
            case 2: {
                setArmPos(Constants.topMotor2, Constants.bottomMotor2);
            }
            case 3: {
                setArmPos(Constants.topMotor3, Constants.bottomMotor3);
            }
            case 4: {
                setArmPos(Constants.topMotor4, Constants.bottomMotor4);
            }
            case 5: {
                setArmPos(Constants.topMotor5, Constants.bottomMotor5);
            }
        }

    }

    void setArmPos(int topMotorPos, int bottomMotorPos){
        atPID.setTargets(topMotorPos, 0.4, 0, 0.2);
        ablPID.setTargets(bottomMotorPos, 0.4, 0, 0.2);
        abrPID.setTargets(bottomMotorPos, 0.4, 0, 0.2);

        robot.abl.setTargetPosition(bottomMotorPos);
        robot.abr.setTargetPosition(bottomMotorPos);
        robot.at.setTargetPosition(topMotorPos);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.at.setPower(0.4);
        robot.abl.setPower(1);
        robot.abr.setPower(1);
    }

    void lowerArm(){
        if (lowerArmCycle){
            atPID.setTargets(constants.topMotorLow, 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorLow, 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorLow, 0.4, 0, 0.2);

            robot.at.setTargetPosition(constants.topMotorLow);

            int somePos = 0;
            if (robot.abl.getCurrentPosition() >= somePos && robot.abl.getCurrentPosition() >= somePos) {
                robot.abl.setTargetPosition(somePos);
                robot.abr.setTargetPosition(somePos);
            } else{
                robot.abl.setTargetPosition(constants.bottomMotorLow);
                robot.abr.setTargetPosition(constants.bottomMotorLow);
            }

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            clawAngle = constants.armServoLow;
            //  robot.armServo.setPosition(constants.armServoLow);

           /*
            robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));
            */

            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);

            if (robot.at.getCurrentPosition() <= constants.topMotorLow + constants.degreeTOLERANCE && robot.abl.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE && robot.abr.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE) lowerArmCycle = false;
        }
    }

    void lowerAllTheWay(){
        if (!lowerArmCycle && lowerAllTheWay){
            clawAngle = constants.armServoBottom;
            atPID.setTargets(constants.topMotorBottom,  0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorBottom,  0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorBottom,  0.4, 0, 0.2);

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

            if (robot.at.getCurrentPosition() <= constants.topMotorBottom + constants.degreeTOLERANCE && robot.abl.getCurrentPosition() <= constants.bottomMotorBottom + constants.degreeTOLERANCE && robot.abr.getCurrentPosition() <= constants.bottomMotorBottom + constants.degreeTOLERANCE) lowerAllTheWay = false;
        }
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



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
//        posSystem.setUpdateGPS(false);
//        gpsUpdateThread.interrupt();
    }
}