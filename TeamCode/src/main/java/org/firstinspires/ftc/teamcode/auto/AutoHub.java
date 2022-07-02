package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

public class AutoHub {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hwMap;

    public AutoHub(LinearOpMode pLinear){
        linearOpMode = pLinear;
        hwMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hwMap);
    }

    public void Spline(){
        while(linearOpMode.opModeIsActive()){
            //xrobot.dtMotors[0].setPower();
        }
    }
}
