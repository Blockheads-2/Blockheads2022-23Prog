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
public class BaseDrive2 extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem = new GlobalPosSystem();
    private ElapsedTime runtime = new ElapsedTime();

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 1.0;
    private double rightThrottle = 1.0;
    private int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
    private int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

    //outputs of joysticks
    double left_stick_x;
    double left_stick_y;
    double right_stick_x;
    double right_stick_y;

    //orientation of joysticks
    double prevLJtargetOrientation = 0.0;
    double LJ_targetOrientation;
    double RJ_targetOrientation;

    //wheel target
    private double wheelOrientation = 0.0;
    double wheelTargetAmountTurned;

    //robot header target
    private double robotOrientation = 0.0;

    RotateSwerveModulePID snapWheelPID;
    RotateSwerveModulePID tableSpinWheelPID;

    //checkover
    private int tolerance = 3; //number of clicks the motors can tolerate to be off by (this number will be determined during testing)
    private boolean stop_snap = false; //checks if the robot is stop_snapping its wheels to a desired orientation

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    private double startingMSsinceRelease;
    private boolean timerHasStarted = false;

    //check for movement
    ElapsedTime movementTime = new ElapsedTime();
    private double lastMovementTime = 0;
    private double prevX = 0;
    private double prevY = 0;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        resetTimer.reset();
        startingMSsinceRelease = resetTimer.milliseconds();

        movementTime.reset();
        lastMovementTime = movementTime.milliseconds();
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void start() { //When "start" is pressed
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
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
        setVariables();
        logic();
        setPower();
    }

    private void setVariables(){
        //outputs of joysticks
        left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]

        //orientation of joysticks
        LJ_targetOrientation = direction(left_stick_x, left_stick_y, wheelOrientation)[1]; //LJ = left joystick
        RJ_targetOrientation = direction(right_stick_x, right_stick_y, robotOrientation)[1];

        //wheel target
        wheelOrientation = posSystem.getPositionArr()[2]; //current orientation of the wheel
        wheelTargetAmountTurned = direction(left_stick_x, left_stick_y, wheelOrientation)[0]; //how much the wheel should turn

        //robot header target
        robotOrientation = posSystem.getPositionArr()[3]; //current orientation of the robot heading

        //module spin power
        spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2));

        //module rotation power
        RotateSwerveModulePID snapWheelPID = new RotateSwerveModulePID(LJ_targetOrientation, 0, 0, 0);
        RotateSwerveModulePID tableSpinWheelPID = new RotateSwerveModulePID(RJ_targetOrientation, 0, 0, 0);
    }


    private void logic(){
        if (LJ_targetOrientation > 90){ //if the wheels must turn more than 90 degrees, stop, stop_snap, move
            stop_snap_move();
        } else{ //otherwise, spline
            //to-do: robot needs to keep a constant header
            spline();
        }

        if (!stop_snap){
            tableSpin();
        }

        prevLJtargetOrientation = LJ_targetOrientation;

        reset(); //snaps wheels back to 0 degrees if the robot has stopped moving
    }

    private void stop_snap_move(){
        boolean finishedTurning = false;

        if (Math.abs(prevLJtargetOrientation - LJ_targetOrientation) > 90){ //if the player moves the joystick more than 90 degrees in an instant
            rightThrottle = 0; //completely stops the entire robot
            leftThrottle = 0;
            stop_snap = true;
            prevX = posSystem.getPositionArr()[0];
            prevY = posSystem.getPositionArr()[1];
            lastMovementTime = movementTime.milliseconds();
        } else if (!isMoving()){ //once the robot has stopped,
            if (Math.abs(wheelOrientation - LJ_targetOrientation) <= tolerance){ //rotate modules until target is hit
                rightThrottle = 1;
                leftThrottle = 1;
                rotationSwitchMotors = ((int)direction(LJ_targetOrientation, wheelOrientation)[1] == 1 ? 1:-1);
                translateSwitchMotors = rotationSwitchMotors;
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
            } else {
                finishedTurning = true;
                stop_snap = false;
            }
            //may need to stop the rotating wheels before proceeding.  Determine this when testing
        } if (finishedTurning){ //run in a linear motion
            translationPowerPercentage = 1.0;
            rotationPowerPercentage = 0.0;
        }
    }

    private void spline(){
        translationPowerPercentage = 1.0;
        rotationPowerPercentage = 0.0;

        double throttle = Math.tanh(Math.abs(left_stick_y / (2 * left_stick_x)));

        if (Math.abs(robotOrientation - LJ_targetOrientation) <= tolerance){
            rightThrottle = ((int)direction(LJ_targetOrientation, wheelOrientation)[1] == 1 ? throttle : 1);
            leftThrottle = ((int)direction(LJ_targetOrientation, wheelOrientation)[1] == 1 ? 1 : throttle);
        } else{
            rightThrottle = 1; //once target is met, stop splining and just move straight
            leftThrottle = 1;
        }
    }

    private void tableSpin(){
        //rotation and translation power percentages
        double tableSpinPower = Math.sqrt(Math.pow(right_stick_x, 2) + Math.pow(right_stick_y, 2));
        rotationSwitchMotors = (int)direction(RJ_targetOrientation, robotOrientation)[1];
        if (Math.abs(RJ_targetOrientation - robotOrientation) <= tolerance){ //while target not hit
            rotationPowerPercentage = tableSpinPower / 1.4; //1.4 can be changed based on testing
            translationPowerPercentage = 1 - rotationPowerPercentage;
        } else{ //after target is hit
            rotationPowerPercentage = 0.0;
            translationPowerPercentage = 1.0;
        }
    }

    private double[] direction(double x, double y, double J_currentOrientation){ //returns how much the robot should turn in which direction
        double[] directionArr = new double[3];
        double switchMotors = 1; //"by default, everything will rotate right."

        double J_targetOrientation = Math.toDegrees(Math.atan2(x, y)); //finds target orientation in terms of degrees (range is (-pi, pi])
        double temp_JtargetOrientation = J_targetOrientation;
        if (J_currentOrientation > J_targetOrientation) J_targetOrientation += 360;
        double targetAmountTurned = Math.abs(J_targetOrientation - J_currentOrientation); //how much the robot/wheel must turn
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //1 = right, -1 = left
        J_targetOrientation = temp_JtargetOrientation;

        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);

        directionArr[0] = targetAmountTurned;
        directionArr[1] = J_targetOrientation;
        directionArr[2] = switchMotors;

        return directionArr;
    }

    private double[] direction(double J_targetOrientation, double J_currentOrientation){
        double[] directionArr = new double[3];
        double switchMotors = 1; //"by default, everything will rotate right."

        double temp = J_targetOrientation;
        if (J_currentOrientation > J_targetOrientation) J_targetOrientation += 360;
        double targetAmountTurned = Math.abs(J_targetOrientation - J_currentOrientation); //how much the robot/wheel must turn
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //1 = right, -1 = left

        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);

        directionArr[0] = targetAmountTurned;
        directionArr[1] = switchMotors;

        return directionArr;
    }

    private void setPower(){
        double[] motorPower = new double[4];

        motorPower[0] = (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * leftThrottle; //top left
        motorPower[1] = (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * leftThrottle; //bottom left
        motorPower[2] = (spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * rightThrottle; //top right
        motorPower[3] = (-1 * spinPower * translationPowerPercentage * translateSwitchMotors + rotatePower * rotationPowerPercentage * rotationSwitchMotors) * rightThrottle ; //bottom right

        if (motorPower[0] == 0 && motorPower[1] == 0 && motorPower[2] == 0 && motorPower[3] == 0){
            robot.setMotorPower(0);
            return;
        }

        for (int i = 0; i <= 3; i++){
            robot.dtMotors[i].setPower(motorPower[i]);
        }
    }

    private void reset(){
        double deltaTime = Math.abs(resetTimer.milliseconds() - startingMSsinceRelease);
        double gapTime = 200; //200 is a placeholder
        double prevOrientation = wheelOrientation-180;
        if (noMovementRequests() && deltaTime > gapTime){ //"If the wheels have stopped for __ milliseconds"
            //make robot's wheels face forward
            if (wheelOrientation != 0){
                rotationSwitchMotors = (wheelOrientation > 0 ? -1 : 1); //1 = right, -1 = left

                RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(0, 0, 0, 0);
                rotatePower = rotateWheelPID.update(wheelOrientation);

                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                leftThrottle = 1.0;
                rightThrottle = 1.0;
                spinPower = 0.0;
                rotatePower = 1.0;
                translateSwitchMotors = 1;
            } else{ //wheels have finished rotating
                timerHasStarted = false;
                robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
                robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                translationPowerPercentage = 0.0; //set everything to 0
                rotationPowerPercentage = 0.0;
                leftThrottle = 0.0;
                rightThrottle = 0.0;
                spinPower = 0.0;
                rotatePower = 0.0;
                rotationSwitchMotors = 0;
                translateSwitchMotors = 0;
            }
        }
    }

    private boolean noMovementRequests(){
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
            if (!timerHasStarted){
                startingMSsinceRelease = resetTimer.milliseconds();
                timerHasStarted = true;
            }
            return true;
        }
        return false;
    }


    private boolean isMoving(){
        double requiredMS = 100.0; //Note: this is ~5 loops

        if (Math.abs(lastMovementTime - movementTime.milliseconds()) >= requiredMS){ //checks if it has been 150 milliseconds
            if (Math.abs(posSystem.getPositionArr()[0] - prevX) <= tolerance && Math.abs(posSystem.getPositionArr()[0] - prevY) <= tolerance){ //if stopped
                return false;
            } else{ //if not stopped
                prevX = posSystem.getPositionArr()[0];
                prevY = posSystem.getPositionArr()[1];
                lastMovementTime = movementTime.milliseconds();
                return true;
            }
        }
        return true;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}