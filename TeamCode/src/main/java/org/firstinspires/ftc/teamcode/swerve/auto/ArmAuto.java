package org.firstinspires.ftc.teamcode.swerve.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.pid.ArmPID;

public class ArmAuto implements Runnable{
    Constants constants = new Constants();
    HardwareDrive robot;

    public enum ArmType{
        HIGH,
        MID,
        LOW,
        GROUND,
        GRAB,
        DROP,
        NONE
    }
    ArmType armType = ArmType.NONE;

    boolean lowerArmCycle = false;
    boolean lowerAllTheWay = false;

    int atTargetPos = 0;
    int ablTargetPos = 0;
    int abrTargetPos = 0;
    double clawAngle = 0;
    double clawClamp = 0;

    double atPower = 0;
    double ablPower = 0;
    double abrPower = 0;

    ArmPID atPID = new ArmPID();
    ArmPID abrPID = new ArmPID();
    ArmPID ablPID = new ArmPID();

    public ArmAuto(HardwareDrive robot){
        this.robot = robot;
    }

    public void run(){
        switch (armType){
            case HIGH:
                atTargetPos = constants.topMotorHigh;
                ablTargetPos = constants.bottomMotorHigh;
                abrTargetPos = constants.bottomMotorHigh;

                atPower = 0.4;
                ablPower = 1;
                abrPower = 1;

                break;

            case MID:
                atTargetPos = constants.topMotorMid;
                ablTargetPos = constants.bottomMotorMid;
                abrTargetPos = constants.bottomMotorMid;

                atPower = 0.4;
                ablPower = 1;
                abrPower = 1;

                break;

            case LOW:
                lowerArmCycle = (robot.at.getCurrentPosition() <= constants.topMotorLow + constants.degreeTOLERANCE && robot.abl.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE && robot.abr.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE);
                lowerArm();

                break;

            case GROUND:
                lowerArmCycle = (robot.at.getCurrentPosition() <= constants.topMotorLow + constants.degreeTOLERANCE && robot.abl.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE && robot.abr.getCurrentPosition() <= constants.bottomMotorLow + constants.degreeTOLERANCE);
                lowerAllTheWay = (robot.at.getCurrentPosition() <= constants.topMotorBottom + constants.degreeTOLERANCE && robot.abl.getCurrentPosition() <= constants.bottomMotorBottom + constants.degreeTOLERANCE && robot.abr.getCurrentPosition() <= constants.bottomMotorBottom + constants.degreeTOLERANCE);

                lowerArm();
                lowerAllTheWay();

                break;

            case GRAB:
                clawClamp = constants.closeClaw;
                break;

            case DROP:
                clawClamp = constants.openClaw;
                break;

            default:

                break;
        }

        robot.at.setTargetPosition(atTargetPos);
        robot.abl.setTargetPosition(ablTargetPos);
        robot.abr.setTargetPosition(abrTargetPos);

        robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        atPID.setTargets(atTargetPos, robot.at.getCurrentPosition(), 0.4, 0, 0.2);
//        ablPID.setTargets(ablTargetPos, robot.abl.getCurrentPosition(), 0.4, 0, 0.2);
//        abrPID.setTargets(abrTargetPos, robot.abr.getCurrentPosition(), 0.4, 0, 0.2);
//        robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));
//        robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
//        robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
        //if you are going to use a PID, you must remember to keep the power running even after the arm has hit its target such that the arm stays in the air.

        robot.at.setPower(atPower);
        robot.abl.setPower(ablPower);
        robot.abr.setPower(abrPower);
    }

    void lowerArm(){
        if (lowerArmCycle){
            clawAngle = constants.armServoLow;

            atTargetPos = constants.topMotorLow;

            int initPos = 0;
            if (robot.abl.getCurrentPosition() >= initPos && robot.abl.getCurrentPosition() >= initPos) {
                ablTargetPos = initPos;
                abrTargetPos = initPos;
            } else{
                ablTargetPos = constants.bottomMotorLow;
                abrTargetPos = constants.bottomMotorLow;
            }
        }
    }

    void lowerAllTheWay(){
        if (!lowerArmCycle && lowerAllTheWay){
            clawAngle = constants.armServoBottom;
            robot.at.setTargetPosition(constants.topMotorBottom);
            robot.abl.setTargetPosition(constants.bottomMotorBottom);
            robot.abr.setTargetPosition(constants.bottomMotorBottom);

            robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.at.setPower(0.4);
            robot.abl.setPower(1);
            robot.abr.setPower(1);

           /*
            robot.abl.setPower(ablPID.update(robot.abl.getCurrentPosition()));
            robot.abr.setPower(abrPID.update(robot.abr.getCurrentPosition()));
            robot.at.setPower(ablPID.update(robot.at.getCurrentPosition()));
            */

        }
    }

    void ClawControl(){
        if (armType == ArmType.GRAB){
            robot.claw.setPosition(0.9);
        } else if (armType == ArmType.DROP){
            robot.claw.setPosition(constants.openClaw);
        }

        if (clawAngle >= 0.7){
            clawAngle = 0.7;
        }
        if (clawAngle <= 0){
            clawAngle = 0;
        }

        robot.armServo.setPosition(clawAngle);
    }

    public ArmType getArmType(){
        return armType;
    }
}
