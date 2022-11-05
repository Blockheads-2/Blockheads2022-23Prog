package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.kinematics.arm.ArmKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Tank Drive", group="Drive")
//@Disabled
public class TankDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Kinematics kinematics;
    Constants constants = new Constants();

    Accelerator acceleratorR = new Accelerator();
    Accelerator acceleratorL = new Accelerator();
    Reset reset;
    double tankPowerR = 0;
    double tankPowerL = 0;

    //buttons
    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();
    Button dpad_right = new Button();
    Button dpad_left = new Button();
    Button clawAngleDownButton = new Button();
    Button clawAngleUpButton = new Button();

    //arm attributes
    ArmKinematics armKinematics = new ArmKinematics();
    int prevPosition;
    private double armPower = 0.8;
    private double clawPosition;

    private enum TelemetryData{
        ARM,
        TANK
    }
    TelemetryData tData = TelemetryData.TANK;
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
        kinematics = new Kinematics(posSystem);
        posSystem.grabKinematics(kinematics);
        reset = new Reset(robot, posSystem);

        telemetry.addData("Say", "Hello Driver");

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        clawPosition = robot.armServo.getPosition();

        robot.armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed
        prevPosition = robot.armBase.getCurrentPosition();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        if (noMovementRequests()){
            reset.reset(true);
        } else {
            reset.reset(false);
            DriveTrainPowerEncoder();
        }
    }

    void UpdatePlayer2(){
        setArmPower();
    }

    void UpdateTelemetry(){
        switch(tData){
            case TANK:
                telemetry.addData("left stick Y", -gamepad1.left_stick_y);
                telemetry.addData("right stick Y", -gamepad1.right_stick_y);
                telemetry.addData("PowerR", tankPowerR);
                telemetry.addData("PowerL", tankPowerL);
                break;

            case ARM:

                break;
        }

        telemetry.addData("Top", robot.armTop.getCurrentPosition());
        telemetry.addData("Bottom", robot.armBase.getCurrentPosition());
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Height", armKinematics.findHeightToGround(armKinematics.getPsi(robot.armBase.getCurrentPosition()), armKinematics.getTheta(robot.armTop.getCurrentPosition())));
        telemetry.addData("Servo Position", robot.claw.getPosition());
        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad2.x);
        y.update(gamepad2.y);
        a.update(gamepad2.a);
        b.update(gamepad2.b);
        clawAngleDownButton.update(gamepad2.left_bumper);
        clawAngleUpButton.update(gamepad2.right_bumper);
        dpad_left.update(gamepad2.dpad_left);
        dpad_right.update(gamepad2.dpad_right);
    }

    /*
    The date is 11/5/2022.

    We are currently in Victor's garage, and the keyboard is very nice.  However, there are no arrow keys and the "/" takes forever to type.

    We are both mentally and physically dying as we are building this little piece of apoijwpeoifjawefawefawe.

    To the future generations, this is a warning from us: cherish your lives! Life is more important than FTC unlesss you're a programer or mechanical person.

     Nevermind, I figured out how to type arrow keys.  You use keys that don't even have an arrow symbol.

     I believe it is within the common interest of the people to sleep.
     */



    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        int distanceTopL = (int)(-gamepad1.right_stick_y * 100);
        int distanceBotL = (int)(gamepad1.right_stick_y * 100);
        int distanceTopR = (int)(-gamepad1.left_stick_y * 100);
        int distanceBotR = (int)(gamepad1.left_stick_y * 100); //idk why, but the joy sticks and the modules are opposites...

        tankPowerL = acceleratorL.update(gamepad1.right_stick_y) * constants.POWER_LIMITER;
        tankPowerR = acceleratorR.update(gamepad1.left_stick_y) * constants.POWER_LIMITER;

        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + distanceBotL);
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceTopL);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + distanceBotR);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + distanceTopR);

        robot.botL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.botL.setPower(tankPowerL);
        robot.topL.setPower(tankPowerL);
        robot.botR.setPower(tankPowerR);
        robot.topR.setPower(tankPowerR);
    }

    public boolean noMovementRequests(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0);
    }


    //________________________ARM METHODS______________________
    void setArmPower(){
        if (y.getState() == Button.State.TAP){
            setTargetPositive();
        }
        if (x.getState() == Button.State.TAP){
            setTargetNegative();
        }
        if(a.getState() == Button.State.TAP){
            setTargetPositiveBase();
        }
        if(b.getState() == Button.State.TAP){
            setTargetNegativeBase();
        }
        if (clawAngleDownButton.getState() == Button.State.TAP){
            clawAngleDown();
        }
        if (clawAngleUpButton.getState() == Button.State.TAP){
            clawAngleUp();
        }
        if (gamepad2.dpad_right){
            coneGrab();
        }
        if (gamepad2.dpad_left){
            coneBack();
        }


        if (robot.armBase.getCurrentPosition() != prevPosition){
            robot.armBase.setTargetPosition(prevPosition);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armBase.setPower(0.1);
        } else if (gamepad1.dpad_left){
            coneBack();
        } else if (robot.armBase.getCurrentPosition() != prevPosition){
            robot.armBase.setTargetPosition(prevPosition);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armBase.setPower(0.1);
        } else {
            robot.armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        prevPosition = robot.armBase.getCurrentPosition();

        if (armPower < 0) armPower = 0;
        else if (armPower > 1) armPower = 1;

        if(clawPosition < 0) clawPosition = 0;
        else if(clawPosition > 1) clawPosition = 1;
    }

    public void setTargetPositive(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armTop.setTargetPosition(topCurrent + 50);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armTop.setPower(armPower);
    }

    public void setTargetNegative(){
        int baseDegree = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        // robot.armBase.setTargetPosition(baseCurrent - 100);
        robot.armTop.setTargetPosition(topCurrent - 50);

        //  robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //    robot.armBase.setPower(0.3);
        robot.armTop.setPower(armPower);
    }

    public void setTargetPositiveBase(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase.setTargetPosition(baseCurrent + 100);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase.setPower(armPower);
    }

    public void setTargetNegativeBase(){
        int baseCurrent = robot.armBase.getCurrentPosition();
        int topCurrent = robot.armTop.getCurrentPosition();

        //  robot.armBase.setTargetPosition(baseCurrent + 100);
        robot.armBase.setTargetPosition(baseCurrent - 100);

        //   robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //  robot.armBase.setPower(0.3);
        robot.armBase.setPower(armPower);
    }

    public void maintainHeightToGroundPositive(){
        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        robot.armBase.setTargetPosition((int)((baseCurrent + (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent - 10));

        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setPower(armPower);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(armPower);
    }

    public void maintainHeightToGroundNegative(){
        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        robot.armBase.setTargetPosition((int)((baseCurrent - (10 * constants.RATIO_CLICKS))));
        robot.armTop.setTargetPosition((int)(topCurrent + 10));

        robot.armBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armBase.setPower(armPower);

        robot.armTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armTop.setPower(armPower);
    }

    public void coneGrab(){
        robot.claw.setPosition(1);
    }

    public void coneBack(){
        robot.claw.setPosition(0.5);
    }

    public void clawAngleUp(){
        clawPosition += 0.1;
        robot.armServo.setPosition(clawPosition);
    }

    public void clawAngleDown(){
        clawPosition -= 0.1;
        robot.armServo.setPosition(clawPosition);
    }

    public void linearExtensionControl(){
        //angles for arm bottomg and top linkages
        double psi = armKinematics.getPsi(robot.armBase.getCurrentPosition());
        double theta = armKinematics.getTheta(robot.armTop.getCurrentPosition());

        double baseCurrent = robot.armBase.getCurrentPosition();
        double topCurrent = robot.armTop.getCurrentPosition();

        int baseTarget = 0;
        int topTarget = 0;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}