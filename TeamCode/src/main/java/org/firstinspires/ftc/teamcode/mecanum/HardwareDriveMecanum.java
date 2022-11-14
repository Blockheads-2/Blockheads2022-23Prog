package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

public class HardwareDriveMecanum {
    //motors
    public DcMotorEx lf;
    public DcMotorEx rf;
    public DcMotorEx lb;
    public DcMotorEx rb;

    public DcMotorEx armBase;
    public DcMotor armTop;

    public Servo claw;
    public Servo armServo;

    public BNO055IMU imu;

    /*Local OpMode Members*/
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();
    Constants constants = new Constants();

    public HardwareDriveMecanum() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        lf  = hwMap.get(DcMotorEx.class, "left_front");
        rf  = hwMap.get(DcMotorEx.class, "right_front");
        lb  = hwMap.get(DcMotorEx.class, "left_back");
        rb  = hwMap.get(DcMotorEx.class, "right_back");
        armBase = hwMap.get(DcMotorEx.class, "arm_base");
        armTop = hwMap.get(DcMotor.class, "arm_top");

        armServo = hwMap.get(Servo.class, "arm_servo");
        claw = hwMap.get(Servo.class, "claw");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        armBase.setDirection(DcMotorEx.Direction.REVERSE);
        armTop.setDirection(DcMotor.Direction.FORWARD);

        claw.setPosition(constants.INITIALIZED_CLAW);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        armBase.setPower(0);
        armTop.setPower(0);

        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armBase.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        armTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
    }
}
