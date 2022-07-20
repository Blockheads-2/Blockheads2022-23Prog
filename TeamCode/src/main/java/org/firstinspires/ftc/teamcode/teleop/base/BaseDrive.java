package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics;
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
    Kinematics kinematics = new Kinematics();

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    private double startingMSsinceRelease;
    private boolean timerHasStarted = false;

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

        setPower();
    }

    private void setVariables(){
        //outputs of joysticks
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]

        //set current orientation
        kinematics.setCurrents(posSystem.getPositionArr()[2], posSystem.getPositionArr()[3]);

        double wheelTurnAmount = kinematics.getWheelDirection(left_stick_x, left_stick_y)[0];
        double robotTurnAmount = kinematics.getRobotDirection(right_stick_x, right_stick_y)[0];

        kinematics.spinPower = Math.sqrt(Math.pow(left_stick_x,2) + Math.pow(left_stick_y, 2));

        if (Math.abs(wheelTurnAmount) > 90){ //if the wheels must turn more than 90 degrees, stop, stop_snap, move
            kinematics.setPos(Kinematics.DriveType.LINEAR, left_stick_x, left_stick_y, wheelTurnAmount, robotTurnAmount);
        } else if (!kinematics.dontspline){ //otherwise, spline
            kinematics.setPos(Kinematics.DriveType.SPLINE, left_stick_x, left_stick_y, wheelTurnAmount, robotTurnAmount);
        }
        kinematics.logic();

        reset(); //snaps wheels back to 0 degrees if the robot has stopped moving
    }

    public double[] getSticksTeleop(double lx, double ly, double rx, double ry) {
        double[] sticks = new double[4];
        sticks[0] = lx;
        sticks[1] = ly;
        sticks[2] = rx;
        sticks[3] = ry;

        return sticks;
    }

    private void setPower(){
        double[] motorPower = new double[4];

        motorPower[0] = kinematics.getPower()[0];
        motorPower[1] = kinematics.getPower()[1];
        motorPower[2] = kinematics.getPower()[2];
        motorPower[3] = kinematics.getPower()[3];

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
        if (noMovementRequests() && deltaTime > gapTime){ //"If the wheels have stopped for __ milliseconds"
            //make robot's wheels face forward
            boolean done = kinematics.resetStuff();
            if (done){
                timerHasStarted = false;
                robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
                robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


//    private boolean isMoving(){
//        double requiredMS = 100.0; //Note: this is ~5 loops
//
//        if (Math.abs(lastMovementTime - movementTime.milliseconds()) >= requiredMS){ //checks if it has been 150 milliseconds
//            if (Math.abs(posSystem.getPositionArr()[0] - prevX) <= constants.TOLERANCE && Math.abs(posSystem.getPositionArr()[0] - prevY) <= constants.TOLERANCE){ //if stopped
//                return false;
//            } else{ //if not stopped
//                prevX = posSystem.getPositionArr()[0];
//                prevY = posSystem.getPositionArr()[1];
//                lastMovementTime = movementTime.milliseconds();
//                return true;
//            }
//        }
//        return true;
//    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}