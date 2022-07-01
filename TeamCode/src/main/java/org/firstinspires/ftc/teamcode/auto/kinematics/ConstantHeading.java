package org.firstinspires.ftc.teamcode.auto.kinematics;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.positioning.MathConstHeadSwerve;

public class ConstantHeading {

    //declaring class objects
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem = new GlobalPosSystem();
    MathConstHeadSwerve constHeadSwerve = new MathConstHeadSwerve();

    //rotation and translate power(for ratio)
    private double translationPowerPercentage = 0.0;
    private double rotationPowerPercentage = 0.0;

    //power of module rotation
    private double rotatePower = 0.0;

    public void constantHeading(double power, double xPosition, double yPosition, double timeoutSeconds){
        //creates new class object for movement
        constHeadSwerve.setFinalPosition(xPosition, yPosition);

        //final distance
        double distance = constHeadSwerve.getDistance();

        //final robot angle, same as module angle
        double theta = constHeadSwerve.getTheta();

        //module turning reference angles
        double previousTheta = 0.0;
        double thetaTurned = Math.abs(theta - previousTheta);

        //switching motor direction
        int switchMotors = 1;

        //check finished turning
        boolean finishedTurning = true;

        //throttling motors
        double rightThrottle = 1;
        double leftThrottle = 1;

        //finds most efficient route for module spinning
        if (previousTheta > theta && theta < 90) theta += 360; //temporarily adds 360
        switchMotors = (theta > previousTheta ? -1 : 1); //determines whether or not the wheel will rotate right or left
        switchMotors *= (thetaTurned <= 90 ? 1 : -1); //determines which way is faster to rotate to get to the desired orientation
        if (thetaTurned > 90) thetaTurned = 90 - (thetaTurned%90);
        theta -= 360;

        //module rotation
        RotateSwerveModulePID rotateWheelPID = new RotateSwerveModulePID(thetaTurned, 0, 0, 0);
        double angleTurned = deltaAngle(theta, thetaTurned);
        rotatePower = rotateWheelPID.update(angleTurned);

        //setting power
        if (thetaTurned <= 90) { //stop, snap, move
            if (Math.abs(previousTheta - theta) > 90) {
                rightThrottle = 0; //completely stops the entire robot.
                leftThrottle = 0;
            }
            rightThrottle = 1;
            leftThrottle = 1;
            if (deltaAngle(theta, thetaTurned) < theta) { //rotate modules until target is hit
                translationPowerPercentage = 0.0;
                rotationPowerPercentage = 1 - translationPowerPercentage;
            } else { //once target is hit, move in linear motion
                translationPowerPercentage = 1.0;
                rotationPowerPercentage = 1 - translationPowerPercentage;
            }
        }
    }

    private double deltaAngle(double theta, double thetaTurned){ //calculates how manya degrees the module has rotated
        double angleTurned = Math.abs(theta - thetaTurned);
        return angleTurned;
    }

    private void setPower(){

    }
}
