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
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.pid.ArmPID;

@TeleOp(name="Final BaseDrive", group="Drive")
//@Disabled
public class FinalBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Thread gpsUpdateThread;
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

    boolean clawClose = false;
    boolean clawUp = false;
    public double clawAngle = 0;
    boolean lowerArmCycle = false;
    boolean lowerAllTheWay = false;

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

        robot.abl.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.abr.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.at.setTargetPosition(0);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
//        robot.abl.setPower(0.7);
//        robot.abr.setPower(0.7);
    }

    @Override
    public void start() { //When "start" is pressed
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.abl.setTargetPosition(robot.abl.getCurrentPosition());
        robot.abr.setTargetPosition(robot.abr.getCurrentPosition());
        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        robot.abl.setPower(0.7); //commented out the arm pos while testing the drivetrain
//        robot.abr.setPower(0.7);

        gpsUpdateThread = new Thread(posSystem);
        gpsUpdateThread.start();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
//        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        DriveTrainPowerEncoder();
    }

    void UpdatePlayer2(){
        ClawControl();
        ArmPresets();
    }

    void UpdateTelemetry(){
//        telemetry.addData("IsAlligned", posSystem.isAlligned());
//        telemetry.addData("Eligible for turning", posSystem.eligibleForTurning());

        telemetry.addData("Leftstick X", gamepad1.left_stick_x);
        telemetry.addData("Leftstick Y", gamepad1.left_stick_y);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("left trigger", gamepad1.left_trigger);


        telemetry.addData("Arm top pos", robot.at.getCurrentPosition());
        telemetry.addData("Arm bot pos", robot.abl.getCurrentPosition());
        telemetry.addData("Arm servo pos", robot.armServo.getPosition());

        telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        telemetry.addData("Left W",  posSystem.getLeftWheelW());
        telemetry.addData("Right W", posSystem.getRightWheelW());
        telemetry.addData("R", posSystem.getPositionArr()[4]);

//        telemetry.addData("Spin Direction (Left)", kinematics.leftThrottle);
//        telemetry.addData("Spin Direction (Right)", kinematics.rightThrottle);
//
//        telemetry.addData("Turn Amount (Left)", kinematics.turnAmountL);
//        telemetry.addData("Turn Amount (Right)", kinematics.turnAmountR);


        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

//        telemetry.addData("Left Spin Clicks Target", kinematics.spinClicksL);
//        telemetry.addData("Left Rotate Clicks target",  kinematics.leftRotClicks);
//        telemetry.addData("Right Spin clicks target", kinematics.spinClicksR);
//        telemetry.addData("Right Rotate clicks target",  kinematics.rightRotClicks);
        telemetry.addData("topL velocity", robot.topL.getVelocity()); //ticks per second
        telemetry.addData("botL velocity", robot.botL.getVelocity()); //ticks per second
        telemetry.addData("topR velocity", robot.topR.getVelocity());
        telemetry.addData("botR velocity", robot.botR.getVelocity());


        telemetry.addData("Power TopL", motorPower[0]);
        telemetry.addData("Power BotL", motorPower[1]);
        telemetry.addData("Power TopR", motorPower[2]);
        telemetry.addData("Power BotR", motorPower[3]);
        telemetry.addData("Clicks Target TopL", targetClicks[0]);
        telemetry.addData("Clicks Target BotL", targetClicks[1]);
        telemetry.addData("Clicks Target TopR", targetClicks[2]);
        telemetry.addData("Clicks Target BotR", targetClicks[3]);

        telemetry.addData("Drive Type", kinematics.getDriveType());

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
    }

    void DriveTrainPowerEncoder(){
        kinematics.logic(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.right_trigger, -gamepad1.left_trigger); //wheelAllignment is one loop late.

        if (kinematics.getDriveType() == RevisedKinematics.DriveType.STOP){
            if (!posSystem.isAlligned(kinematics.PodR.getSpinDirection(), kinematics.PodL.getSpinDirection()) || x.getState() == Button.State.TAP){
                reset.reset(true);
            }
            return;
        } else {
            reset.reset(false);
        }

        targetClicks = kinematics.getClicks();
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower = kinematics.getPower();
        robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
        robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
        robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
        robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);
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
        gpsUpdateThread.interrupt();
    }
}