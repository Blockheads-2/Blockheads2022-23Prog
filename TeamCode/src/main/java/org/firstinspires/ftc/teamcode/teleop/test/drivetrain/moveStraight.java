//package org.firstinspires.ftc.teamcode.teleop.test.drivetrain;
//
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.common.Accelerator;
//import org.firstinspires.ftc.teamcode.common.Reset;
//import org.firstinspires.ftc.teamcode.common.Button;
//import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
//
//import org.firstinspires.ftc.teamcode.common.HardwareDrive;
//import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
//
//@TeleOp(name="Move Straight", group="Drive")
////@Disabled
//public class moveStraight extends OpMode{
//    /* Declare OpMode members. */
//    HardwareDrive robot = new HardwareDrive();
//    Constants constants = new Constants();
//
//    public enum TelemetryType{
//        POWER,
//        CONTROLLER
//    }
//    TelemetryType tType = TelemetryType.CONTROLLER;
//
//    Reset reset;
//    Accelerator accelerator = new Accelerator();
//    LinearCorrectionPID linearCorrectionPIDRight;
//    LinearCorrectionPID linearCorrectionPIDLeft;
//
//    int targetClicks;
//    int errorCorrectionClicksL;
//    int errorCorrectionClicksR;
//    double power;
//
//    Button x = new Button();
//    Button y = new Button();
//    Button a = new Button();
//    Button b = new Button();
//
//    /** The relativeLayout field is used to aid in providing interesting visual feedback
//     * in this sample application; you probably *don't* need this when you use a color sensor on your
//     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
//    View relativeLayout;
//
//    @Override
//    public void init() { //When "init" is clicked
//        robot.init(hardwareMap);
////        reset = new Reset(robot);
//
//        linearCorrectionPIDRight = new LinearCorrectionPID();
//        linearCorrectionPIDLeft = new LinearCorrectionPID();
//        linearCorrectionPIDRight.setValues(0, 0.03, 0, 0.01);
//        linearCorrectionPIDLeft.setValues(0, 0.03, 0, 0.01);
//
//        telemetry.addData("Say", "Hello Driver");
//    }
//
//    @Override
//    public void init_loop() { //Loop between "init" and "start"
//        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    @Override
//    public void start() { //When "start" is pressed
//
//    }
//
//    @Override
//    public void loop() { //Loop between "start" and "stop"
//        UpdatePlayer1();
//        UpdatePlayer2();
//        UpdateButton();
//        UpdateTelemetry();
//    }
//
//    void UpdatePlayer1(){
//        if (x.getState() == Button.State.TAP){
//            reset.reset(true);
//        } else {
//            DriveTrainMove();
//            reset.reset(false);
//        }
//
//        if (a.getState() == Button.State.TAP){
//            tType = TelemetryType.CONTROLLER;
//        } else if (b.getState() == Button.State.TAP){
//            tType = TelemetryType.POWER;
//        }
//    }
//
//    void UpdatePlayer2(){
//    }
//
//    void UpdateTelemetry(){
//
//        switch (tType){
//            case CONTROLLER:
//                telemetry.addData("X gamepad", gamepad1.left_stick_x);
//                telemetry.addData("Y gamepad", -gamepad1.left_stick_y);
//                telemetry.addData("Orientation", Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)));
//                telemetry.addData("topL Clicks", robot.topL.getCurrentPosition());
//                telemetry.addData("botL Clicks", robot.botL.getCurrentPosition());
//                telemetry.addData("topR Clicks", robot.topR.getCurrentPosition());
//                telemetry.addData("botR Clicks", robot.botR.getCurrentPosition());
//                break;
//
//            case POWER:
//                telemetry.addData("Accelerated Power", power);
//                telemetry.addData("Target Clicks", targetClicks);
//                break;
//        }
//
//
//        telemetry.update();
//    }
//
//    void UpdateButton(){
//        x.update(gamepad1.x);
//        y.update(gamepad1.y);
//        a.update(gamepad1.a);
//        b.update(gamepad1.b);
//    }
//
//    private void DriveTrainMove(){
//        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
//        double left_stick_y = -gamepad1.left_stick_y; //returns a value between [-1, 1]
//        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
//        double right_stick_y = -gamepad1.right_stick_y; //returns a value between [-1, 1]
//
//        errorCorrectionClicksL = correctError()[0];
//        errorCorrectionClicksR = correctError()[1];
//
//        targetClicks = (int) left_stick_y * 100;
//
//        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks - errorCorrectionClicksL);
//        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - targetClicks - errorCorrectionClicksL);
//        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks + errorCorrectionClicksR);
//        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - targetClicks + errorCorrectionClicksR);
//
//        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        power = left_stick_y;
//        power *= constants.POWER_LIMITER;
//        telemetry.addData("Limited power", power);
//        power = accelerator.update(power);
//
//        robot.topL.setPower(-power);
//        robot.botL.setPower(-power);
//        robot.topR.setPower(power);
//        robot.botR.setPower(power);
//
//        if (power == 0) {
//            robot.setMotorPower(0);
//        }
//    }
//
//    public int[] correctError(){
//        int topR = robot.topR.getCurrentPosition();
//        int botR = robot.botR.getCurrentPosition();
//        int topL = robot.topL.getCurrentPosition();
//        int botL = robot.botL.getCurrentPosition();
//
//        int rotateR = (topR + botR) / 2;
//        int rotateL = (topL + botL) / 2;
//
//
//
//        rotateR %= (constants.CLICKS_PER_PURPLE_REV);
//        rotateL %= constants.CLICKS_PER_PURPLE_REV;
//
////        rotateR = (int)(20 * linearCorrectionPIDRight.update(rotateR));
////        rotateL = (int)(20 * linearCorrectionPIDLeft.update(rotateL)); //can try both this and the two lines above.
//
//        int[] error = {-rotateL, -rotateL};
//
//        rotateR *= constants.DEGREES_PER_CLICK;
//        rotateL *= constants.DEGREES_PER_CLICK;
//
//        telemetry.addData("Right error", -rotateR);
//        telemetry.addData("Left error", -rotateL);
//
//        return error;
//    }
//
//    public boolean noMovementRequests(){
//        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0);
//    }
//
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }
//}