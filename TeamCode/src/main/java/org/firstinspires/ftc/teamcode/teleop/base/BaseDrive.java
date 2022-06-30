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

    //Used for tracking the robot's orientation and how much it needs to turn to return to it's original position
    private double orientation = 0.0;

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;

    //module's orientation
    private double targetOrientation = 0.0;
    private double previousOrientation = 0.0;
    private int switchMotors = 1;

    //conditions
    private boolean stutter = false;

    //checkover
    private boolean finishedTurning = true;

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
        constantHeading();
    }

    private void constantHeading(){
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double targetOrientation = Math.atan2(left_stick_x, left_stick_y);
        double previousOrientation = 0.0;
        double targetAmountTurned = Math.abs(targetOrientation - previousOrientation);

        if (previousOrientation > targetOrientation && targetOrientation < 90) targetOrientation += 360; //temporarily adds 360
        switchMotors = (targetOrientation > previousOrientation ? -1 : 1); //determines whether or not the wheel will rotate right or left
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //determines which way is faster to rotate to get to the desired orientation
        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);
        targetOrientation -= 360;

        //
        if (finishedTurning){
            botRstartingClick = robot.dtMotors[3].getCurrentPosition();
            topRstartingClick = robot.dtMotors[2].getCurrentPosition();
        }
        finishedTurning = (Math.abs(deltaAngle()) >= Math.abs(targetAmountTurned));

        //module spin power
        spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2));

        //module rotation power
        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(targetAmountTurned, 0, 0, 0);
        double angleTurned = deltaAngle() * constants.DEGREES_PER_CLICK;
        orientation += angleTurned;
        rotatePower = rotateWheelPID.update(angleTurned);

        if (targetAmountTurned <= 90){ //stop, snap, move
            robot.setMotorPower(0); // <-- this needs to be fixed, because it'll stop the movement every loop

            if (deltaAngle() < targetOrientation){ //rotate modules until target is hit
                robot.dtMotors[3].setPower(rotatePower * switchMotors);
                robot.dtMotors[1].setPower(rotatePower * switchMotors);
                robot.dtMotors[2].setPower(rotatePower * switchMotors);
                robot.dtMotors[0].setPower(rotatePower * switchMotors);
            } else{ //once target is hit, move in linear motion
                robot.dtMotors[3].setPower(-1 * spinPower * switchMotors);
                robot.dtMotors[1].setPower(-1 * spinPower * switchMotors);
                robot.dtMotors[2].setPower(spinPower * switchMotors);
                robot.dtMotors[0].setPower(spinPower * switchMotors);
            }
        } else{ //spline
            double throttle = Math.tanh(Math.abs(left_stick_y / (2 * left_stick_x)));
            double botRightMotorPower = -1 * spinPower * switchMotors * throttle;
            double topRightMotorPower = spinPower  * switchMotors * throttle;
            double topLeftMotorPower = -1 * spinPower * switchMotors;
            double botLeftMotorPower = spinPower * switchMotors;

            if (switchMotors == -1){ //checks if robot needs to turn left (default is turning right)
                botRightMotorPower = -1 * spinPower * switchMotors;
                topRightMotorPower = spinPower  * switchMotors;
                topLeftMotorPower = -1 * spinPower * switchMotors * throttle;
                botLeftMotorPower = spinPower * switchMotors * throttle;
            }
        }
        previousOrientation = targetOrientation;
    }

    private void spin(){
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]
        double angle = Math.atan2(right_stick_x, right_stick_y);
    }


    private double deltaAngle(){ //calculates how many clicks were allocated to rotating the module
        /*
        based on the number of clicks the motor has ran, this method figures out how much it has turned.
         */
        double clicksTOP = Math.abs(robot.dtMotors[2].getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.dtMotors[3].getCurrentPosition() - botRstartingClick);

        double clicksSpun = (clicksTOP-clicksBOT) / 2;
        double clicksRotated = Math.abs(clicksTOP) - clicksSpun;

        return clicksRotated;
    }

    private double deltaSpun(){ //calculates how many clicks were allocated to spinning the module
        double clicksTOP = Math.abs(robot.dtMotors[2].getCurrentPosition() - topRstartingClick);
        double clicksBOT = Math.abs(robot.dtMotors[3].getCurrentPosition() - botRstartingClick);

        double clicksSpun = (clicksTOP-clicksBOT) / 2;
        return clicksSpun;
    }

    private boolean wheelsAreStopped(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0);
    }

    private void reset(){
        double deltaTime = Math.abs(resetTimer.milliseconds() - startingMilliseconds);
        double gapTime = 200; //200 is a placeholder
        if (wheelsAreStopped() && deltaTime > gapTime){ //"If the wheels have stopped for __ milliseconds"
            robot.resetEncoders();
            robot.runUsingEncoders();
            //make robot's wheels face forward
            /*
            Ideas:
            - stores how much the robot has turned since the last reset(). Based on this number, the robot will know how much it has to turn right/left to face forward
             */
            if (orientation > 90){
                orientation = 90 - (orientation %= 90);

            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
