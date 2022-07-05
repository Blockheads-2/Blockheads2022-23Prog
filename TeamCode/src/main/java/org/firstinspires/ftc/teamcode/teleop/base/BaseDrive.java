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

    //orientation
    private double robotOrientation = 0.0;
    private double wheelOrientation = 0.0;

    //robot's power
    private double rotatePower = 0.0;
    private double spinPower = 0.0;
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;
    private double leftThrottle = 0.0;
    private double rightThrottle = 0.0;
    private double switchMotors = 1.0;

    //checkover
    private boolean finishedTurning = true;

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
        startingMilliseconds = resetTimer.milliseconds();

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
        drive();
        setPower();
    }



    private void drive(){
        //output of left joystick
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]

        //wheel orientation
        wheelOrientation = posSystem.getPositionArr()[2]; //current orientation of the wheel
        double prevOrientation = 0.0; //previous orientation of the wheel before it begins a turning cycle

        //orientation of joystick & amount wheel must turn
        double prevTargetOrientation = 0.0;
        double targetAmountTurned = robotDirection(left_stick_x, left_stick_y, wheelOrientation)[0];
        double targetOrientation = robotDirection(left_stick_x, left_stick_y, wheelOrientation)[1];

        //direction robot turns
        switchMotors = (int)robotDirection(left_stick_x, left_stick_y, wheelOrientation)[2];

        //checks if
        if (finishedTurning) prevOrientation = wheelOrientation;
        finishedTurning = (deltaAngle(prevOrientation, wheelOrientation) >= targetAmountTurned);

        //module spin power
        spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2));

        //module rotation power
        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(targetAmountTurned, 0, 0, 0);
        double angleTurned = deltaAngle(prevOrientation, wheelOrientation);
        rotatePower = rotateWheelPID.update(angleTurned);

        if (targetAmountTurned > 90){ //stop, snap, move
            rightThrottle = 1;
            leftThrottle = 1;
            if (Math.abs(prevTargetOrientation - targetOrientation) > 90){ //if the player moves the joystic more than 90 degrees in an instant
                robot.setMotorPower(0); //completely stops the entire robot
            } else if (!finishedTurning){ //rotate modules until target is hit
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
            } else{ //once target is hit, move in linear motion
                translationPowerPercentage = 1.0;
                rotationPowerPercentage = 0.0;
            }
        } else{ //spline
            //to-do: robot needs to stop splining after it hits its target
            spline(left_stick_x, left_stick_y);
        }
        if (Math.sqrt(Math.pow(posSystem.getPositionArr()[0],2) + Math.pow(posSystem.getPositionArr()[1], 2)) != 0){
            //tableSpin() should only occur when the robot is translating
            tableSpin();
        }

        prevTargetOrientation = targetOrientation;
    }

    private void spline(double x, double y){
        boolean finishedTurning;

        translationPowerPercentage = 1.0;
        rotationPowerPercentage = 0.0;

        double throttle = Math.tanh(Math.abs(y / (2 * x)));
        rightThrottle = (switchMotors == 1 ? throttle : 1);
        leftThrottle = (switchMotors == 1 ? 1 : throttle);
        //if switchMotors = 1, then robot is turning right. If switchMotors = -1, then robot is turning left.
    }

    private void tableSpin(){
        //output of right joystick
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]

        //robot's orientation
        robotOrientation = posSystem.getPositionArr()[3];
        double prevOrientation = 0.0;

        //right joystick's orientation
        double targetAmountTurned = robotDirection(right_stick_x, right_stick_y, robotOrientation)[0];
        double targetOrientation = robotDirection(right_stick_x, right_stick_y, robotOrientation)[1];
        switchMotors = (int)robotDirection(right_stick_x, right_stick_y, robotOrientation)[2];

        //checks if
        if (finishedTurning) prevOrientation = robotOrientation;
        finishedTurning = (deltaAngle(prevOrientation, robotOrientation) >= targetAmountTurned);

        //rotate power:
        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(targetAmountTurned, 0, 0, 0);
        double angleTurned = deltaAngle(prevOrientation, robotOrientation);
        rotatePower = rotateWheelPID.update(angleTurned);

        //rotation and translation power percentages
        double tableSpinPower = Math.sqrt(Math.pow(right_stick_x, 2) + Math.pow(right_stick_y, 2));
        rotationPowerPercentage = tableSpinPower / 1.4; //1.4 can be changed based on testing
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

    private double deltaAngle(double prevOrientation, double currentOrientation){ //calculates how many degrees the module has rotated
        double angleTurned = Math.abs(currentOrientation - prevOrientation);

        if (angleTurned > 90) angleTurned = 90 - (angleTurned%90);

        return angleTurned;
    }

    private void reset(){
        double deltaTime = Math.abs(resetTimer.milliseconds() - startingMilliseconds);
        double gapTime = 200; //200 is a placeholder
        double prevOrientation = wheelOrientation-180;
        if (wheelsAreStopped() && deltaTime > gapTime){ //"If the wheels have stopped for __ milliseconds"
            //make robot's wheels face forward
            if (wheelOrientation != 0){
                finishedTurning = false;
                double amountNeededToTurn = wheelOrientation - 180; //makes it [-180, 180] and tells us if the wheels need to turn right or left
                switchMotors = (amountNeededToTurn > 0 ? -1 : 1); //1 = right, -1 = left

                RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(amountNeededToTurn, 0, 0, 0);
                double angleTurned = deltaAngle(prevOrientation, wheelOrientation-180);
                rotatePower = rotateWheelPID.update(angleTurned);

                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1.0;
                leftThrottle = 1.0;
                rightThrottle = 1.0;
                spinPower = 0.0;
                rotatePower = 1.0;
                setPower();
            }
            finishedTurning = true;
            robot.setMotorPower(0);

            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private boolean wheelsAreStopped(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0);
    }

    private double[] robotDirection(double x, double y, double currentOrientation){ //returns how much the robot should turn in which direction
        double[] directionArr = new double[3];

        int switchMotors = -1; //"by default, the robot's wheels will rotate left."

        currentOrientation = posSystem.getPositionArr()[2];
        double targetOrientation = Math.atan2(x, y);
        targetOrientation = Math.toDegrees(targetOrientation) + 180; //converts radians to degrees and adds 180 to make the rage [0,360]
        double temp_targetOrientation = targetOrientation;
        if (currentOrientation > targetOrientation) targetOrientation += 360;
        double targetAmountTurned = Math.abs(targetOrientation - currentOrientation);
        switchMotors *= (targetAmountTurned <= 90 ? 1 : -1); //determines if rotating right or left is faster to get to the desired orientation

        if (targetAmountTurned > 90) targetAmountTurned = 90 - (targetAmountTurned%90);
        targetOrientation = temp_targetOrientation;

        directionArr[0] = targetAmountTurned;
        directionArr[1] = targetOrientation;
        directionArr[2] = switchMotors;

        return directionArr;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
