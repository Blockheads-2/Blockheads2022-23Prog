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
import org.firstinspires.ftc.teamcode.swerve.common.pid.HeaderControlPID;
import java.lang.Math;

@TeleOp(name="AAAAAA Test Teleop", group="Drive")
//@Disabled
public class TestTeleop extends OpMode{
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
    int armPos = 0;
    int armPos2 = 0;


    double xvalue = 40;
    double yvalue = 40;

    double topSegLength = 416; //406
    double botSegLength = 420; //420

    Constants constants = new Constants();
    Reset reset;

    SwervePod PodL = new SwervePod(constants.initDirectionLeft, SwervePod.Side.LEFT);
    SwervePod PodR = new SwervePod(constants.initDirectionRight, SwervePod.Side.RIGHT);

    private enum TelemetryData{
        LEFT,
        RIGHT
    }
    TelemetryData tData = TelemetryData.LEFT;

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

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        double angleTarget = robot.armServo.getPosition();
        robot.armServo.setPosition(angleTarget);

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

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION); //leave it in RUN_TO_POSITION for the entirety

        loopTime.reset();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
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
//        ArmPresets();
//        armPosTesting();
        xvalue += 50*(-gamepad2.left_stick_y);
        yvalue += 50*(-gamepad2.right_stick_y);
        triangle();
    }

    void UpdateTelemetry(){
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Leftstick Y", -gamepad1.left_stick_y);
        telemetry.addData("Rightstick Y", -gamepad1.right_stick_y);

        telemetry.addData("Arm top pos", robot.at.getCurrentPosition());
        telemetry.addData("Arm bot pos", robot.abl.getCurrentPosition());
        telemetry.addData("Arm servo pos", robot.armServo.getPosition());


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

//        outputL = PodL.getOutput();
//        outputR = PodR.getOutput();
//
//        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + (int)(outputL[0] + outputL[1]));
//        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + (int)(-outputL[0] + outputL[1]));
//        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + (int)(outputR[0] + outputR[1]));
//        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + (int)(-outputR[0] + outputR[1]));
//
//        robot.topL.setVelocity(outputL[2] * outputL[3] * constants.MAX_VELOCITY_DT);
//        robot.botL.setVelocity(outputL[2] * outputL[3] * constants.MAX_VELOCITY_DT);
//        robot.topR.setVelocity(outputR[2] * outputR[3] * constants.MAX_VELOCITY_DT);
//        robot.botR.setVelocity(outputR[2] * outputR[3] * constants.MAX_VELOCITY_DT);

//        robot.topL.setPower(outputL[2] * outputL[3]);
//        robot.botL.setPower(outputL[2] * outputL[3]);
//        robot.topR.setPower(outputR[2] * outputR[3]);
//        robot.botR.setPower(outputR[2] * outputR[3]);

        deltaMS = loopTime.milliseconds() - prevMS;
        prevMS = loopTime.milliseconds();
    }

    void ArmPresets(){
        robot.at.setPower(0.4);
        robot.abl.setPower(1);
        robot.abr.setPower(1);

        if (bottomButton.is(Button.State.TAP)){
            lowerArmCycle = true;
            lowerAllTheWay = true;
            updateArmPos();
        }

        if (lowButton.is(Button.State.TAP)){
            lowerArmCycle = true;
            updateArmPos();
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
            updateArmPos();

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
            updateArmPos();

            //robot.armServo.setPosition(constants.armServoHigh);

            /*robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));

             */
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

    void armPosTesting(){
        if (highButton.is(Button.State.HELD)) {
            armPos = armPos + 10;
        }
        if (bottomButton.is(Button.State.HELD)) {
            armPos = armPos - 10;
        }
        if (midButton.is(Button.State.HELD)) {
            armPos2 = armPos2 + 10;
        }
        if (lowButton.is(Button.State.HELD)) {
            armPos2 = armPos2 - 10;
        }
        setArmPos(armPos, armPos2);
    }

    void triangle(){
        double z = Math.sqrt((xvalue*xvalue)+(yvalue*yvalue));

        if (z>800){
            double tempx = xvalue;
            double tempy = yvalue;
            xvalue = 800*(Math.cos(Math.atan(tempx/tempy)));
            yvalue = 800*(Math.sin(Math.atan(tempx/tempy)));
        }

        if (z<75){
            xvalue = 150;
            yvalue = 150;
        }

        if (xvalue < 0){
            xvalue = 0;
        }
        if (yvalue < 150){
            yvalue = 150;
        }

        z = Math.sqrt((xvalue*xvalue)+(yvalue*yvalue));

        double topMotorAngle = Math.toDegrees(Math.acos(((botSegLength*botSegLength)+(topSegLength*topSegLength)-(z*z))/(2*(botSegLength)*(topSegLength))));
        double bottomMotorAngle = Math.toDegrees(Math.acos(((z*z)+(topSegLength*topSegLength)-(botSegLength*botSegLength))/(2*(z)*(topSegLength))));
        double finalBottomarmangle = 180-(bottomMotorAngle + Math.toDegrees(Math.asin(yvalue/z)));
        double servoAngleChange = (180 - (topMotorAngle-finalBottomarmangle))-90;

        double servoUnits = 0.6-(servoAngleChange/constants.anglePerUnit);
        int topMotorClicks = (int)(topMotorAngle/constants.topMotorAnglePerClick)-37;
        int bottomMotorClicks = (int)(finalBottomarmangle/constants.bottomMotorAnglePerClick)-632;
        telemetry.addData("Bottom Target Clicks", bottomMotorClicks);
        telemetry.addData("Top Target Clicks", topMotorClicks);
        telemetry.addData("Bottom Target Angle", finalBottomarmangle);
        telemetry.addData("Top Target Angle", topMotorAngle);
        telemetry.addData("Top Arm Relative to the Robot", 180 - (topMotorAngle-finalBottomarmangle));
        telemetry.addData("Servo Target Angle", servoAngleChange);
        telemetry.addData("Servo Target Units", servoUnits);

        telemetry.addData("z", z);
        telemetry.addData("x", xvalue);
        telemetry.addData("y", yvalue);



        if ((yvalue > 0) && (xvalue > 0)){
            robot.at.setTargetPosition(topMotorClicks);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setPower((60/76.0));

            robot.abl.setTargetPosition(bottomMotorClicks);
            robot.abr.setTargetPosition(bottomMotorClicks);
            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abl.setPower(0.7);
            robot.abr.setPower(0.7);
            robot.armServo.setPosition(servoUnits);
        }


    }

    void updateArmPos(){
        double topArm = robot.at.getCurrentPosition();
        double bottomArm = robot.abl.getCurrentPosition();

        double topArmAngle = (topArm*constants.topMotorAnglePerClick) + 6.088534;
        double bottomArmAngle  = (bottomArm*constants.bottomMotorAnglePerClick) * 43.067047;

        xvalue = botSegLength*Math.cos(bottomArmAngle) - topSegLength*Math.cos(-bottomArmAngle-topArmAngle);
        yvalue = botSegLength*Math.sin(bottomArmAngle) - topSegLength*Math.sin(-bottomArmAngle-topArmAngle);
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