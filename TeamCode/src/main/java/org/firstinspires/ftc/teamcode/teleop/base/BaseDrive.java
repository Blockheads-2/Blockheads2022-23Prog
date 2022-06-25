package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.pid.TurnPID;

@TeleOp(name="Base Drive", group="Drive")
//@Disabled
public class BaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    private boolean toggleButton = true;

    private double botRstartingClick = 0;
    private double topRstartingClick = 0;

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
        robot.resetEncoders();
        robot.runUsingEncoders();
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){

        DriveTrainBase();
        //OscillateServo();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);
      //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void UpdateButton(){
    }

    void DriveTrainBase(){
        DriveTrainMove();
    }

    private void DriveTrainMove(){
        double x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double power = Math.sqrt(Math.pow(x,2) + Math.pow(y, 2)); //the power inputted (x and y are already unit vecotrs)
        double angle = Math.atan2(x, y);

        boolean wheelHasFinishedRotating;

        TurnPID rotateWheelPID = new TurnPID(angle, 0, 0, 0);
        double angleCorrection;

        int switchMotors = (angle <= 90 ? 1:-1); // is the the correct way to use a ternary expression?
        int botL_ENC_target;
        int botR_ENC_target;
        int topL_ENC_target;
        int topR_ENC_target;

        if (x != 0){ //rotates wheels (and only wheels), and then moves
            wheelHasFinishedRotating = false;

            if (switchMotors == -1){
                angle = 90 - (angle %= 90);
                /*
                Example:
                If the wheels need to turn 150 degrees right, it's faster to turn 30 degrees left and then switch the direction the motor turns.

                According to the equation above, 90 - (150%90) = 90 - 60 = 30
                 */
            }
            botL_ENC_target = robot.botL.getCurrentPosition() + (int)(angle * constants.BOT_CLICKS_PER_DEGREE);
            botR_ENC_target = robot.botR.getCurrentPosition() + (int)(angle * constants.BOT_CLICKS_PER_DEGREE);
            topL_ENC_target = robot.topL.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE);
            topR_ENC_target = robot.topR.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE);

            robot.botL.setTargetPosition(botL_ENC_target);
            robot.botR.setTargetPosition(botR_ENC_target);
            robot.topL.setTargetPosition(topL_ENC_target);
            robot.topR.setTargetPosition(topR_ENC_target);

            botRstartingClick = robot.botR.getCurrentPosition();
            topRstartingClick = robot.topR.getCurrentPosition();

            robot.runToEncoderPosition();

            while(wheelHasFinishedRotating == false){
                angleCorrection = rotateWheelPID.update(deltaAngle(botRstartingClick, topRstartingClick));
                robot.botR.setPower(1 * angleCorrection * switchMotors);
                robot.botL.setPower(1 * angleCorrection * switchMotors);
                robot.topR.setPower(1 * angleCorrection * switchMotors); //goal is to turn really fast and accurately
                robot.topL.setPower(1 * angleCorrection * switchMotors);

                if (!robot.botL.isBusy() && !robot.botR.isBusy() && !robot.topL.isBusy() && !robot.topR.isBusy()) wheelHasFinishedRotating = true; //checks if the wheels have finished rotating to the desired angle
            }
            robot.setMotorPower(0); //Do we need this???  ? ?? ? ? ?? ? ? ?? ?? ? ?? ? ? ?? ?
        }

        robot.botR.setPower(-power * switchMotors); //makes the robot move forward/backwards
        robot.botL.setPower(-power * switchMotors);
        robot.topR.setPower(power * switchMotors);
        robot.topL.setPower(power * switchMotors);
        //top gear of swerve module goes in opposite direction as the bottom gear for the wheel to spin
    }

    private double deltaAngle(double botRstartingClick, double topRstartingClick){
        /*
        based on the number of clicks the motor has ran, we will figure out how much it has turned.
         */
        double clicksTOP = Math.abs(robot.topR.getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.botR.getCurrentPosition() - botRstartingClick);
        //multiply these by whatever ratio to determine how much the robot has turned
        double angleTurned = 0; //0 is a placeholder

        return angleTurned;
    }

    private boolean checkForActivity(){

    }

    private void reset(){
        if (checkForActivity() == false){ //"If the gamepad has not been touched for __ milliseconds"
            robot.resetEncoders();
            robot.runUsingEncoders();
            //make robot's wheels face forward
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
