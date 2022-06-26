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

    //used for determining how much the robot's wheels have rotated
    private double botRstartingClick = 0.0;
    private double topRstartingClick = 0.0;

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    private double startingMilliseconds;
    private double totalTurned = 0.0; //a measure of how much the wheels have turned right

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        resetTimer.reset();
        startingMilliseconds = resetTimer.milliseconds();

    }

    @Override
    public void init_loop() {
        robot.resetEncoders();
        robot.runUsingEncoders();
    }

    @Override
    public void start() {
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

        botRstartingClick = robot.botR.getCurrentPosition();
        topRstartingClick = robot.topR.getCurrentPosition();

        if (x != 0){ //rotates wheels (and only wheels), and then moves
            wheelHasFinishedRotating = false;

            if (switchMotors == -1){
                angle = 90 - (angle %= 90);
                /*
                Example:
                If the wheels need to turn 150 degrees right, it's faster to turn 30 degrees left and then switch the direction the motors turn.

                According to the equation above, turning 150 degrees right = 90 - (150%90) = 90 - 60 = 30
                 */
            }
            botL_ENC_target = robot.botL.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE); //turning both gears in the same direction
            botR_ENC_target = robot.botR.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE); //at the same speed rotates the wheel
            topL_ENC_target = robot.topL.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE); //without rotating the robot.
            topR_ENC_target = robot.topR.getCurrentPosition() + (int)(angle * constants.TOP_CLICKS_PER_DEGREE); //Using 'TOP' because I'm assuming that top is faster. It's probably wrong, so do math later

            robot.botL.setTargetPosition(botL_ENC_target * switchMotors);
            robot.botR.setTargetPosition(botR_ENC_target * switchMotors);
            robot.topL.setTargetPosition(topL_ENC_target * switchMotors);
            robot.topR.setTargetPosition(topR_ENC_target * switchMotors);

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

    private double deltaAngle(double botRstartingClick, double topRstartingClick){ //calculates how much a wheel turned
        /*
        based on the number of clicks the motor has ran, we will figure out how much it has turned.
         */
        double clicksTOP = Math.abs(robot.topR.getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.botR.getCurrentPosition() - botRstartingClick);
        //multiply these by whatever ratio to determine how much the robot has turned
        double angleTurned = 0; //0 is a placeholder

        return angleTurned;
    }

    private boolean checkForWheelMovement(){
        return (botRstartingClick == robot.botR.getCurrentPosition() && topRstartingClick == robot.topL.getCurrentPosition());
    }

    private void reset(){
        double deltaTime = Math.abs(resetTimer.milliseconds() - startingMilliseconds);
        double the_amount_of_time_the_robot_must_wait_before_it_is_told_that_it_is_not_moving = 200; //200 is a placeholder. Decide time later.
        if (checkForWheelMovement() == false && deltaTime >= the_amount_of_time_the_robot_must_wait_before_it_is_told_that_it_is_not_moving){ //"If the robot has not moved for __ milliseconds"
            robot.resetEncoders();
            robot.runUsingEncoders();
            //make robot's wheels face forward
            /*
            Ideas:
            - stores how much the robot has turned since the last reset(). Based on this number, the robot will know how much it has to turn right/left to face forward
             */
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
