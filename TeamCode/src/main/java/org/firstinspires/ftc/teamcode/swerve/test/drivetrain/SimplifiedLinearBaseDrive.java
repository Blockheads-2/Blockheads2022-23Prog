package org.firstinspires.ftc.teamcode.swerve.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.SimplifiedKinematics;

@TeleOp(name="Simplified Linear Base Drive Test", group="Drive")
@Disabled
public class SimplifiedLinearBaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    SimplifiedKinematics kinematics;

    Reset reset;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    public enum TelemetryType{
        CLICKS,
        POWER
    }
    TelemetryType tType = TelemetryType.POWER;

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        kinematics = new SimplifiedKinematics(posSystem);
//        posSystem.grabKinematics(kinematics);
        //as long as you don't call posSystem.getDriveType, the posSystem does not need to grab the kinematics.
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");
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
        if (a.getState() == Button.State.TAP){
            tType = TelemetryType.POWER;
        } else if (b.getState() == Button.State.TAP){
            tType = TelemetryType.CLICKS;
        }

        if(x.getState() == Button.State.TAP){
            kinematics.switchRotateDirection();
        } else if (y.getState() == Button.State.TAP){
            kinematics.switchSpinDirection();
        }

        setVariables();

        if (kinematics.getDriveType() == SimplifiedKinematics.DriveType.STOP){
            reset.reset(true);
        } else{
            reset.reset(false);
            DriveTrainBase();
        }
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        double[] posData = posSystem.getPositionArr();

        switch(tType){
            case POWER:
                telemetry.addData("X gamepad", gamepad1.left_stick_x);
                telemetry.addData("Y gamepad", -gamepad1.left_stick_y);
                telemetry.addData("Target", kinematics.getTarget());
                telemetry.addData("Right Turn Amount", kinematics.getRTurnAmount());
                telemetry.addData("Left Turn Amount", kinematics.getLTurnAmount());
                telemetry.addData("Right Direction", kinematics.getRightDirectionW());
                telemetry.addData("Left Direction", kinematics.getLeftDirectionW());
                telemetry.addData("SpinR Direction", kinematics.spinDirectionR);
                telemetry.addData("SpinL Direction", kinematics.spinDirectionL);
                telemetry.addData("Should Snap?", kinematics.shouldSnap());
                telemetry.addData("Reset?", (kinematics.getDriveType() == SimplifiedKinematics.DriveType.STOP));
                telemetry.addData("DriveType?", kinematics.getDriveType());
                break;

            case CLICKS:
                double[] motorPower = kinematics.getPower();

                telemetry.addData("X", posData[0]);
                telemetry.addData("Y", posData[1]);
                telemetry.addData("Left W", posData[2]);
                telemetry.addData("Right W", posData[3]);
                telemetry.addData("R", posData[4]);
                telemetry.addData("Power Top", motorPower[2]);
                telemetry.addData("Power Bottom", motorPower[3]);
                telemetry.addData("topL Clicks", robot.topL.getCurrentPosition());
                telemetry.addData("botL Clicks", robot.botL.getCurrentPosition());
                break;
        }
        
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
        //put power into the motors
        setPower();
    }

    private void setVariables(){
        posSystem.calculatePos();
        //outputs of joysticks
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = -gamepad1.left_stick_y; //returns a value between [-1, 1]
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = -gamepad1.right_stick_y; //returns a value between [-1, 1]

        kinematics.getGamepad(left_stick_x, left_stick_y, right_stick_x, right_stick_y);

        kinematics.logic();
    }

    private void setPower(){
        int[] targetClicks = kinematics.getClicks();
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double[] motorPower = kinematics.getPower();
        robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
        robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
        robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
        robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}