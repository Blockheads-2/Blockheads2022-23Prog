package org.firstinspires.ftc.teamcode.swerve.teleop.base;

import android.view.View;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.Reset;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.Button;

import org.firstinspires.ftc.teamcode.swerve.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.swerve.common.kinematics.SwervePod;
import org.firstinspires.ftc.teamcode.swerve.common.pid.ArmPID;

@TeleOp(name="Final BaseDrive", group="Drive")
//@Disabled
public class FinalBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Thread gpsUpdateThread;
    RevisedKinematics kinematics;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ElapsedTime loopTime = new ElapsedTime();
    double prevMS = 0;
    double deltaMS = 0;

    Constants constants = new Constants();
    Reset reset;

    SwervePod PodL = new SwervePod(constants.initDirectionLeft, SwervePod.Side.LEFT);
    SwervePod PodR = new SwervePod(constants.initDirectionRight, SwervePod.Side.RIGHT);

    Button resetHeader = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    int[] targetClicks = new int[4];
    double[] motorPower = new double[4];

    //ARM ATTRIBUTES
    Button x = new Button();
    Button bottomButton = new Button();
    Button lowButton = new Button();
    Button midButton = new Button();
    Button highButton = new Button();

    Button zeroButton = new Button();
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

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    int loopCount = 0;
    double avgVelTopL = 0;
    double avgVelBotL = 0;
    double avgVelTopR = 0;
    double avgVelBotR = 0;

    double avgDifferenceBetweenTops = 0;
    double avgDifferenceBetweenBottoms = 0;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);

        posSystem = new GlobalPosSystem(robot);

        kinematics = new RevisedKinematics(posSystem, PodL, PodR);
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);
        kinematics.grabTelemetry(telemetry);

        moveArmToInit();

        telemetry.addData("Say", "Hello Driver");

//        gpsUpdateThread = new Thread(posSystem);
//        gpsUpdateThread.start();
    }

    public void moveArmToInit(){
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.abl.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.abr.setTargetPosition(constants.INIT_ARMBASE_POS);
        robot.at.setTargetPosition(0);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.abl.setPower(0.7);
        robot.abr.setPower(0.7);
        robot.at.setPower(0.7);

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
        robot.abr.setTargetPosition(robot.at.getCurrentPosition());
        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.abl.setPower(0.7);
        robot.abr.setPower(0.7);
        robot.at.setPower(0.7);

        robot.setWheelRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelRunMode(DcMotor.RunMode.RUN_TO_POSITION); //leave it in RUN_TO_POSITION for the entirety

        loopTime.reset();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        loopCount++;
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        if (resetHeader.getState() == Button.State.DOUBLE_TAP) posSystem.resetHeader();
        if (a.getState() == Button.State.TAP){
            posSystem.resetXY();
        }
        DriveTrainPowerEncoder();
    }

    void UpdatePlayer2(){
        ClawControl();
        ArmPresets();
        UltraMegaArmPresets();
    }

    void UpdateTelemetry(){
        telemetry.addData("Conventional Joystick Power (Left joystick)", Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(-gamepad1.left_stick_y, 2)));
        telemetry.addData("Init Pole L?", PodL.getPole());
        telemetry.addData("Init Pole R?", PodR.getPole());

        telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        telemetry.addData("Optimized Left W", PodL.getOptimizedCurrentW());
        telemetry.addData("Optimized Right W", PodR.getOptimizedCurrentW());
        telemetry.addData("R", posSystem.getPositionArr()[4]);

//        telemetry.addData("Throttle (Left)", PodL.getThrottle());
//        telemetry.addData("Throttle (Right)", PodR.getThrottle());

        telemetry.addData("topL AVERAGE velocity", avgVelTopL / loopCount); //ticks per second
        telemetry.addData("botL AVERAGE velocity", avgVelBotL / loopCount); //ticks per second
        telemetry.addData("topR AVERAGE velocity", avgVelTopR / loopCount); //ticks per second
        telemetry.addData("botR AVERAGE velocity", avgVelBotR / loopCount); //ticks per second
        telemetry.addData("avg ratio between top velocities (Left : Right)", (avgVelTopL / loopCount) / (avgVelTopR / loopCount)); //ticks per second
        telemetry.addData("avg ratio between bottom velocities (Left : Right)", (avgVelBotL / loopCount) / (avgVelBotR / loopCount)); //ticks per second

        telemetry.addData("Drive Type", kinematics.getDriveType());

        telemetry.addData("PIDF Coefficients topL", robot.topL.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("PIDF Coefficients botL", robot.botL.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("PIDF Coefficients topR", robot.topR.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("PIDF Coefficients botR", robot.botR.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("TopL Target Pos Tolerance", robot.topL.getTargetPositionTolerance());
        telemetry.addData("TopR Target Pos Tolerance", robot.topR.getTargetPositionTolerance());

        telemetry.addData("Delta time loop (sec)", deltaMS / 1000.0);

        telemetry.update();
    }

    void UpdateButton(){
        resetHeader.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);

        x.update(gamepad2.x);
        bottomButton.update(gamepad2.dpad_down);
        lowButton.update(gamepad2.dpad_right);
        midButton.update(gamepad2.dpad_left);
        highButton.update(gamepad2.dpad_up);
        zeroButton.update(gamepad2.b);
        leftBumpy.update(gamepad2.left_bumper);
        rightBumpy.update(gamepad2.right_bumper);
    }

    void DriveTrainPowerEncoder(){
        kinematics.logic(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.right_trigger, -gamepad1.left_trigger); //wheelAllignment is one loop late.

        targetClicks = kinematics.getClicks();
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + (int)(targetClicks[0]));
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + (int)(targetClicks[1]));
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + (int)(targetClicks[2]));
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + (int)(targetClicks[3]));

        motorPower = kinematics.getPower();
        robot.topL.setVelocity(motorPower[0] * constants.MAX_VELOCITY_DT);
        robot.botL.setVelocity(motorPower[1] * constants.MAX_VELOCITY_DT);
        robot.topR.setVelocity(motorPower[2] * constants.MAX_VELOCITY_DT);
        robot.botR.setVelocity(motorPower[3] * constants.MAX_VELOCITY_DT);

        deltaMS = loopTime.milliseconds() - prevMS;
        prevMS = loopTime.milliseconds();

        avgVelTopL += robot.topL.getVelocity();
        avgVelBotL += robot.botL.getVelocity();
        avgVelTopR += robot.topR.getVelocity();
        avgVelBotR += robot.botR.getVelocity();
    }

    void ArmPresets(){
        robot.at.setPower(0.4);
        robot.abl.setPower(1);
        robot.abr.setPower(1);

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



            clawAngle = constants.armServoMid;
            //robot.armServo.setPosition(constants.armServoMid);

            /*robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

             */
        }

        if (highButton.is(Button.State.TAP)){
            atPID.setTargets(constants.topMotorHigh, 0.4, 0, 0.2);
            ablPID.setTargets(constants.bottomMotorHigh, 0.4, 0, 0.2);
            abrPID.setTargets(constants.bottomMotorHigh, 0.4, 0, 0.2);

            robot.abl.setTargetPosition(constants.bottomMotorHigh);
            robot.abr.setTargetPosition(constants.bottomMotorHigh);
            robot.at.setTargetPosition(constants.topMotorHigh);

            clawAngle = constants.armServoHigh;
            //robot.armServo.setPosition(constants.armServoHigh);

            /*robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

             */
        }
    }

    void UltraMegaArmPresets(){
        if (leftBumpy.is(Button.State.TAP)){
            stackClawPos--;
            if (stackClawPos<1){
                stackClawPos = 1;
            }
            if (stackClawPos == 1) {
                setArmPos(constants.topMotor1, constants.bottomMotor1);
            }
            else if (stackClawPos == 2) {
                setArmPos(constants.topMotor2, constants.bottomMotor2);
            }
            else if (stackClawPos == 3) {
                setArmPos(constants.topMotor3, constants.bottomMotor3);
                robot.armServo.setPosition(0.1514);
            }
            else if (stackClawPos == 4) {
                setArmPos(constants.topMotor4, constants.bottomMotor4);
                robot.armServo.setPosition(0.1514);
            }
            else if (stackClawPos == 5) {
                setArmPos(constants.topMotor5, constants.bottomMotor5);
            }
        }
        if (rightBumpy.is(Button.State.TAP)){
            stackClawPos++;
            if (stackClawPos>5){
                stackClawPos = 5;
            }
            if (stackClawPos == 1) {
                setArmPos(constants.topMotor1, constants.bottomMotor1);
            }
            else if (stackClawPos == 2) {
                setArmPos(constants.topMotor2, constants.bottomMotor2);
            }
            else if (stackClawPos == 3) {
                setArmPos(constants.topMotor3, constants.bottomMotor3);
            }
            else if (stackClawPos == 4) {
                setArmPos(constants.topMotor4, constants.bottomMotor4);
            }
            else if (stackClawPos == 5) {
                setArmPos(constants.topMotor5, constants.bottomMotor5);
            }
        }

    }

    void setArmPos(int topMotorPos, int bottomMotorPos){
        //atPID.setTargets(topMotorPos, 0.4, 0, 0.2);
        //ablPID.setTargets(bottomMotorPos, 0.4, 0, 0.2);
        //abrPID.setTargets(bottomMotorPos, 0.4, 0, 0.2);

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