package org.firstinspires.ftc.teamcode.common.kinematics;

import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;

public class MathVariableHeadPhi {
    private double angle;

    MathSpline mathSpline = new MathSpline();
    void setVars(){
        angle = mathSpline.returnTheta();
    }

    RotateSwerveModulePID pid = new RotateSwerveModulePID();
}
