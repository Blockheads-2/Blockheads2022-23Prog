package org.firstinspires.ftc.teamcode.auto.kinematics;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHeadSwerve;

public class ConstantHeading {
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    MathConstHeadSwerve constHeadSwerve = new MathConstHeadSwerve();

    public void constantHeading(double xPosition, double yPosition, double power){
        //final position
        constHeadSwerve.setFinalPosition(xPosition, yPosition);
        double distance = constHeadSwerve.getDistance();

        //module angle
        double theta = constHeadSwerve.getTheta();
        double previousTheta = 0.0;
        double thetaTurned = Math.abs(theta - previousTheta);

        //switching motor direction
        int switchMotors = 1;

        //check finished turning
        boolean finishedTurning = true;

        //finds most efficient route for module spinning
        if (previousTheta > theta && theta < 90) theta += 360; //temporarily adds 360
        switchMotors = (theta > previousTheta ? -1 : 1); //determines whether or not the wheel will rotate right or left
        switchMotors *= (thetaTurned <= 90 ? 1 : -1); //determines which way is faster to rotate to get to the desired orientation
        if (thetaTurned > 90) thetaTurned = 90 - (thetaTurned%90);
        theta -= 360;


    }
}
