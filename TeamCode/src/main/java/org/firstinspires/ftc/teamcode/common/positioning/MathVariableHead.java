package org.firstinspires.ftc.teamcode.common.positioning;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

public class MathVariableHead<y, x> {
    MathSpline mathSpline = new MathSpline();

    //Variables inputted
    private double x; //Horizontal Movement in Inches
    private double y; //Vertical Movement in Inches
    private int finalAngle; //end Angle Orientation (relative to start pos)

    //variablesOutputted
    int[] motorClicks = new int[4]; //arr[0] = lA, arr[1] = lB, arr[2] = rA, arr[3] = rB


    public void setVariables(int xChange, int yChange, int endAngle){
        x = xChange;
        y = yChange;
        finalAngle = endAngle;
    }

    mathSpline.setFinalPose(x,y);

    public int[] returnClicks(){
        double rDistance = mathSpline.returnRDistance() * 32.382;
        rightMotorClicks[2] = (int) (rDistance);
        rightMotorClicks[3] = (int) (-rDistance);

        double lDistance = mathSpline.returnLDistance() * 32.382;
        leftMotorClicks[0] = (int) (lDistance);
        leftMotorClicks[1] = (int) (-lDistance);

        return motorClicks;
    }
}
