package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
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
    GlobalPosSystem posSystem = new GlobalPosSystem();
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

    //module's orientation
    private int switchMotors = 1;
    private double currentOrientation = 0.0;
    private double startingOrienation = 0.0;

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 0.0;
    private double rightThrottle = 0.0;

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
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void DriveTrainBase(){
        DriveTrainMove();
    }

    private void DriveTrainMove(){
        posSystem.calculatePos();
        constantHeading();
        setPower();
    }

    private void constantHeading(){
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double targetOrientation = Math.atan2(left_stick_x, left_stick_y);
        double prevTargetOrientation = 0.0;
        double temp_targetOrientation = targetOrientation;
        currentOrientation = posSystem.getPositionArr()[2];

        if (currentOrientation > targetOrientation) targetOrientation += 360;
        switchMotors = -1; //"by default, the robot's wheels will rotate left."

        double targetAmountTurned = Math.abs(targetOrientation - currentOrientation);
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //determines if rotating right or left is faster to get to the desired orientation
        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);
        targetOrientation = temp_targetOrientation;
        if (finishedTurning) startingOrienation = currentOrientation;
        finishedTurning = (deltaAngle() >= targetAmountTurned);

        //module spin power
        spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2));

        //module rotation power
        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(targetAmountTurned, 0, 0, 0);
        double angleTurned = deltaAngle();
        rotatePower = rotateWheelPID.update(angleTurned);

        if (targetAmountTurned <= 90){ //stop, snap, move
            if (Math.abs(prevTargetOrientation - targetOrientation) > 90){
                rightThrottle = 0; //completely stops the entire robot.
                leftThrottle = 0;
            }
            rightThrottle = 1;
            leftThrottle = 1;
            if (deltaAngle() < targetOrientation){ //rotate modules until target is hit
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1 - translationPowerPercentage;
            } else{ //once target is hit, move in linear motion
                translationPowerPercentage = 1.0;
                rotationPowerPercentage = 1 - translationPowerPercentage;
            }
        } else{ //spline
            translationPowerPercentage = 1.0;
            rotationPowerPercentage = 1 - translationPowerPercentage;

            double throttle = Math.tanh(Math.abs(left_stick_y / (2 * left_stick_x)));
            rightThrottle = (switchMotors == 1 ? throttle : 1);
            leftThrottle = (switchMotors == 1 ? 1 : throttle);
            //if switchMotors = 1, then robot is turning right. If switchMotors = -1, then robot is turning left.
        }
        prevTargetOrientation = targetOrientation;
    }
    private void tableSpin(){
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]
        double tableSpinPower = Math.sqrt(Math.pow(right_stick_x, 2) + Math.pow(right_stick_y, 2));
        double targetOrientation = Math.atan2(right_stick_x, right_stick_y);
        double temp_targetOrientation =  targetOrientation;
        double robotOrientation = posSystem.getPositionArr()[3];
        double angleSpun = Math.abs(targetOrientation - currentOrientation);

        if (currentOrientation > targetOrientation) targetOrientation += 360;
        switchMotors = -1; //"by default, the robot's wheels will rotate left."
        //Note: Must take into account the switchMotors set in constantHeading()

        double targetAmountTurned = Math.abs(targetOrientation - currentOrientation);
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //determines if rotating right or left is faster to get to the desired orientation
        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);
        targetOrientation = temp_targetOrientation;
        if (finishedTurning) startingOrienation = currentOrientation;
        finishedTurning = (deltaAngle() >= targetAmountTurned);

        /*
         spinPower * translationPowerPercentage +
         rotatePower * rotationPowerPercentage
         */

        translationPowerPercentage = 1 - rotationPowerPercentage;

    }

    private void setPower(){
        double topRMotoRPower = (spinPower * translationPowerPercentage + rotatePower * rotationPowerPercentage) * rightThrottle * switchMotors;
        double botRMotorPower = (-1 * spinPower * translationPowerPercentage + rotatePower * rotationPowerPercentage) * rightThrottle * switchMotors ;
        double topLMotorPower = (spinPower * translationPowerPercentage + rotatePower * rotationPowerPercentage) * leftThrottle * switchMotors;
        double botLMotorPower = (-1 * spinPower * translationPowerPercentage + rotatePower * rotationPowerPercentage) * leftThrottle * switchMotors;

        robot.dtMotors[0].setPower(topLMotorPower);
        robot.dtMotors[1].setPower(botLMotorPower);
        robot.dtMotors[2].setPower(topRMotoRPower);
        robot.dtMotors[3].setPower(botRMotorPower);
    }

    private double deltaAngle(){ //calculates how many degrees the module has rotated
        double angleTurned = Math.abs(startingOrienation - currentOrientation);
        return angleTurned;
    }


    private boolean wheelsAreStopped(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0);
    }

    private void reset(){
        double deltaTime = Math.abs(resetTimer.milliseconds() - startingMilliseconds);
        double gapTime = 200; //200 is a placeholder
        if (wheelsAreStopped() && deltaTime > gapTime){ //"If the wheels have stopped for __ milliseconds"
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
