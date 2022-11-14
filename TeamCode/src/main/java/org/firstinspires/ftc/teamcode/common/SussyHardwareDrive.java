package org.firstinspires.ftc.teamcode.common;

import android.app.Activity;
import android.hardware.Sensor;
import android.view.View;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.Range;

public class SussyHardwareDrive
{
    //Motors
    public DcMotorEx  lf   = null;
    public DcMotorEx  rf   = null;
    public DcMotorEx  lb   = null;
    public DcMotorEx  rb   = null;

    //Sensor
    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    SussyConstants constants = new SussyConstants();

    /* Constructor */
    public SussyHardwareDrive(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotorEx.class, "left_front");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rb = hwMap.get(DcMotorEx.class, "right_back");

        //IMU initiation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Reverse Motor
        lf.setDirection(DcMotorEx.Direction.FORWARD);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
