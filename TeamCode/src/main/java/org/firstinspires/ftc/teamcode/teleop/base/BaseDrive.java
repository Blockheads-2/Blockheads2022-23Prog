package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

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

    void DriveTrainBase(){DriveTrainMove();}

    private void DriveTrainMove(){
        //constant heading
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double angle = Math.atan2(left_stick_x, left_stick_y);

        //spin robot's orientation
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]

        //spline
        double right_trigger = gamepad1.right_trigger; //returns a value between [0, 1]
        double left_trigger = gamepad1.left_trigger; //returns a value between [0, 1]


        //power
        double spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2)); //the power inputted (x and y are already unit vecotrs)
        double rotatePower;
        double translationPowerPercentage = 0.0; //0.5 is a placeholder.
        double rotationPowerPercentage = 0.0;

        //checkover
        boolean wheelHasFinishedRotating = true;

        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(angle, 0, 0, 0);

        int switchMotors = (angle <= 90 ? 1:-1);

        if (wheelHasFinishedRotating){
            botRstartingClick = robot.botR.getCurrentPosition();
            topRstartingClick = robot.topR.getCurrentPosition();
        }

        if (left_stick_x != 0 || right_stick_x != 0 || right_stick_y != 0 || right_trigger != 0 || left_trigger != 0){ //rotates wheels (and only wheels), and then moves
            wheelHasFinishedRotating = false;
            if (switchMotors == -1){
                angle = 90 - (angle %= 90);
                /*
                Example:
                If the wheels need to turn 150 degrees right, it's faster to turn 30 degrees left and then switch the direction the motors turn.

                According to the equation above, turning 150 degrees right = 90 - (150%90) = 90 - 60 = 30
                 */
            }
            translationPowerPercentage += (left_stick_x == 0 ?  : 0);
            translationPowerPercentage += (right_stick_x == ? : 0);
            translationPowerPercentage += (right_stick_y == ? : 0);

            rotationPowerPercentage = 1.0 - translationPowerPercentage;
        }

        double angleTurned = deltaAngle() * constants.DEGREES_PER_CLICK;
        rotatePower = rotateWheelPID.update(angleTurned);

        double botMotorPower = (-1 * translationPowerPercentage * spinPower + translationPowerPercentage * rotatePower) * switchMotors;
        double topMotorPower = (translationPowerPercentage * spinPower + translationPowerPercentage * rotatePower) * switchMotors;

        robot.botR.setPower(botMotorPower);
        robot.botL.setPower(botMotorPower);
        robot.topR.setPower(topMotorPower);
        robot.topL.setPower(topMotorPower);
    }

    private double deltaAngle(){ //calculates how many clicks were allocated to rotating the module
        /*
        based on the number of clicks the motor has ran, this method figures out how much it has turned.
         */
        double clicksTOP = Math.abs(robot.topR.getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.botR.getCurrentPosition() - botRstartingClick);

        double clicksSpun = (clicksTOP-clicksBOT) / 2;
        double clicksRotated = Math.abs(clicksTOP) - clicksSpun;

        return clicksRotated;
    }

    private double deltaSpun(){ //calculates how many clicks were allocated to spinning the module
        double clicksTOP = Math.abs(robot.topR.getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.botR.getCurrentPosition() - botRstartingClick);

        double clicksSpun = (clicksTOP-clicksBOT) / 2;
        return clicksSpun;
    }

    private boolean checkForWheelMovement(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y && );
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
