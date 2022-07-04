package org.firstinspires.ftc.teamcode.auto.kinematics;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

public class LinearMotionVariableHeading {
    //declaring class objects
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem = new GlobalPosSystem();

    ////rotation and translate power(for ratio)
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;

    //power of module rotation
    private double rotatePower = 0.0;

    private double x = 0;
    private double y = 0;

    public void linearMotionVariableHeading(){

    }

}
