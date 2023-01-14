package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.RevisedKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.SwervePod;

import java.util.HashMap;

public class GlobalPosSystem {

    Constants constants = new Constants();
    RevisedKinematics revisedKinematics;

    private double[] positionArr = new double[5];

    private int[] motorClicksPos = new int[4];
    private int[] prevMotorClicks = new int[4];

    HardwareDrive robot;

    public double optimizedCurrentWR = 0;
    public double optimizedCurrentWL = 0;

    Orientation lastOrientation;
    Orientation currentOrientation;
    double currAngle = 0;

    private boolean updateGPS = true;

    public GlobalPosSystem(HardwareDrive robot) {
        this.robot = robot;

        motorClicksPos[0] = robot.topL.getCurrentPosition();
        motorClicksPos[1] = robot.botL.getCurrentPosition();
        motorClicksPos[2] = robot.topR.getCurrentPosition();
        motorClicksPos[3] = robot.botR.getCurrentPosition();

        prevMotorClicks[0] =  motorClicksPos[0];
        prevMotorClicks[1] =  motorClicksPos[1];
        prevMotorClicks[2] =  motorClicksPos[2];
        prevMotorClicks[3] =  motorClicksPos[3];

        currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void grabKinematics(RevisedKinematics k){
        revisedKinematics = k;
    }

    public void calculatePos(){
        updateHash();
        calculateWheel();
        calculateHeader();
        positionArr[2] = clamp(positionArr[2] +  positionArr[4]);
        positionArr[3] = clamp(positionArr[3] + positionArr[4]);
        calculateRobot();
    }

    public void calculateWheel(){
        double rotateL = (robot.topL.getCurrentPosition() + robot.botL.getCurrentPosition()) / 2.0; //total rotation of left module
        rotateL *= constants.DEGREES_PER_CLICK;
        double rotateR = (robot.topR.getCurrentPosition() + robot.botR.getCurrentPosition()) / 2.0; //total rotation of right module
        rotateR *= constants.DEGREES_PER_CLICK;

        positionArr[2] = clamp(rotateL);
        positionArr[3] = clamp(rotateR);
    }

    public void calculateHeader(){ //add this after implementing the extension hub onto the robot
        // Get current orientation
        currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = currentOrientation.firstAngle - lastOrientation.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle <= -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle -= deltaAngle;
        lastOrientation = currentOrientation;
        positionArr[4] = clamp(currAngle);
    }

    public void calculateRobot(){
        //right
        int topR = motorClicksPos[2] - prevMotorClicks[2]; //change in top right
        int botR = motorClicksPos[3] - prevMotorClicks[3]; //change in bottom right
        double translationalInchesR = (topR - botR) / 2.0;
        translationalInchesR *= constants.INCHES_PER_CLICK;
        translationalInchesR *= constants.initDirectionRight;
        double currentAngleR = positionArr[3];

        //left
        int topL = motorClicksPos[0] - prevMotorClicks[0]; //change in top left
        int botL = motorClicksPos[1] - prevMotorClicks[1]; //change in bottom left
        double translationalInchesL = (topL - botL) / 2.0;
        translationalInchesL *= constants.INCHES_PER_CLICK;
        translationalInchesL *= constants.initDirectionLeft;
        double currentAngleL = positionArr[2];

        double splineOrientation = 0.0;
        double baseAngle = (currentAngleL + currentAngleR) / 2.0;
        baseAngle = Math.toRadians(baseAngle);
        double hypotenuse = (translationalInchesL + translationalInchesR) / 2.0;

//        if (revisedKinematics.getDriveType() == RevisedKinematics.DriveType.SPLINE){
//            double bigArc = Math.max(translationalInchesL, translationalInchesR); //unit: inches
//            double smallArc = Math.min(translationalInchesL, translationalInchesR); //unit: inches
//            if (Math.abs(bigArc - smallArc) <= 0.1){
//                double radius = ((bigArc + smallArc) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) / (bigArc - smallArc); //unit: inches
//                double theta = (bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER); //unit: radians
//                hypotenuse = Math.sqrt((2 * radius * radius) * (1 - Math.cos(theta)));
//                splineOrientation = Math.toDegrees(theta);
//                baseAngle = (Math.PI - theta) / 2.0; //unit: radians
//                baseAngle = (Math.PI / 2.0) - baseAngle;
//            } //problem: this assumes that the modules are parallel.
//        } else if (revisedKinematics.getDriveType() == RevisedKinematics.DriveType.TURN){
//            positionArr[2] += positionArr[4];
//            positionArr[3] += positionArr[4];
//        }

        positionArr[0] += (hypotenuse * Math.sin(baseAngle + positionArr[4]));
        positionArr[1] += (hypotenuse * Math.cos(baseAngle + positionArr[4]));
    }

    public void update ( double x, double y, double leftWheelW, double rightWheelW, double robotR){
        //update
        positionArr[0] += (x);
        positionArr[1] += (y);
        positionArr[2] += leftWheelW;
        positionArr[3] += rightWheelW;
        positionArr[4] += robotR;

        positionArr[2] = clamp(positionArr[2]);
        positionArr[3] = clamp(positionArr[3]);
        positionArr[4] = clamp(positionArr[4]);
    }

    public double getLeftWheelW(){
        return positionArr[2];
    }

    public double getRightWheelW(){
        return positionArr[3];
    }

    public boolean isAlligned(){
        double error = SwervePod.changeAngle(optimizedCurrentWL, optimizedCurrentWR);

        return (Math.abs(error) < constants.allignmentTolerance);
    }

    public boolean resetWorthy(){
        double error = SwervePod.changeAngle(optimizedCurrentWL, optimizedCurrentWR);
        return (Math.abs(error) < constants.degreeTOLERANCE);
    }

    public void setOptimizedCurrentW(double angleR, double angleL){
        optimizedCurrentWR = angleR;
        optimizedCurrentWL = angleL;
    }

    public boolean eligibleForTurning(){
        return (Math.abs(optimizedCurrentWL) <= constants.allignmentTolerance &&
                Math.abs(optimizedCurrentWR) <= constants.allignmentTolerance &&
                isAlligned());
    }

    public void hardResetGPS(){
        positionArr[0] = 0; //reset x, y position as well to account for drift.
        positionArr[1] = 0;
//        positionArr[2]=positionArr[4];
//        positionArr[3]=positionArr[4];
        positionArr[2]=0;
        positionArr[3]=0;

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorClicksPos[0] = robot.topL.getCurrentPosition();
        motorClicksPos[1] = robot.botL.getCurrentPosition();
        motorClicksPos[2] = robot.topR.getCurrentPosition();
        motorClicksPos[3] = robot.botR.getCurrentPosition();

        prevMotorClicks[0] =  motorClicksPos[0];
        prevMotorClicks[1] =  motorClicksPos[1];
        prevMotorClicks[2] =  motorClicksPos[2];
        prevMotorClicks[3] =  motorClicksPos[3];
    }

    public void resetXY(){
        positionArr[0] = 0;
        positionArr[1] = 0;
    }

    public void updateHash(){
        prevMotorClicks[0] =  motorClicksPos[0];
        prevMotorClicks[1] =  motorClicksPos[1];
        prevMotorClicks[2] =  motorClicksPos[2];
        prevMotorClicks[3] =  motorClicksPos[3];

        motorClicksPos[0] = robot.topL.getCurrentPosition();
        motorClicksPos[1] = robot.botL.getCurrentPosition();
        motorClicksPos[2] = robot.topR.getCurrentPosition();
        motorClicksPos[3] = robot.botR.getCurrentPosition();
    }

    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
        clicks[0] = robot.topL.getCurrentPosition();
        clicks[1] = robot.botL.getCurrentPosition();
        clicks[2] = robot.topR.getCurrentPosition();
        clicks[3] = robot.botR.getCurrentPosition();

        return clicks;
    }

    public HashMap<String, Double> getArmClicks() {
        HashMap<String, Double> armClicks = new HashMap<String, Double>();
        armClicks.put("at", (double)robot.at.getCurrentPosition());
        armClicks.put("abl", (double)robot.abl.getCurrentPosition());
        armClicks.put("abr", (double)robot.abr.getCurrentPosition());
        armClicks.put("armServo", robot.armServo.getPosition());
        armClicks.put("claw", robot.claw.getPosition());

        return armClicks;
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;
        if (degrees == -180) degrees = 180;

        if (degrees < -180){
            degrees = 180 - (Math.abs(degrees) - 180);
        } else if (degrees > 180){
            degrees = -180 + (Math.abs(degrees) - 180);
        }
        return degrees;
    }

//    public void run(){
//        while (updateGPS){
//            calculatePos();
//        }
//    }

    public void setUpdateGPS(boolean gpsthread){
        updateGPS = gpsthread;
    }

    public RevisedKinematics.DriveType getDriveType() {
        return revisedKinematics.getDriveType();
    }
}