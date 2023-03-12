package org.firstinspires.ftc.teamcode.swerve.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.pid.SnapSwerveModulePID;

public class Reset {
    HardwareDrive robot;
    GlobalPosSystem globalPosSystem;
    Constants constants = new Constants();
    Accelerator accelerator = new Accelerator();

    //PIDs
    SnapSwerveModulePID snapLeftWheelPID;
    SnapSwerveModulePID snapRightWheelPID;

    double power = 0;
    double powerL = 0; //for auto
    double powerR = 0;

    boolean isResetCycle = false;
    boolean resetDone = false;

    public Reset(HardwareDrive r, GlobalPosSystem gps){
        robot = r;
        globalPosSystem=gps;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();
        snapLeftWheelPID.setTargets(constants.kpRotation, constants.kiRotation, constants.kdRotation);
        snapRightWheelPID.setTargets(constants.kpRotation, constants.kiRotation, constants.kdRotation);
    }

    public void reset(boolean shouldReset){
        if (shouldReset){
            updateReset();
        } else{
            isResetCycle = false;
            resetDone = false;
        }
    }

    public void resetAuto(boolean shouldReset){
        if (shouldReset){
            updateResetAuto();
        } else{
            isResetCycle = false;
            resetDone = false;
        }
    }

    private void updateReset(){
        globalPosSystem.calculatePos();
        int rotateL = -(int)(globalPosSystem.getLeftWheelW() * constants.CLICKS_PER_DEGREE); //total rotation of left module
        int rotateR = (int)(globalPosSystem.getRightWheelW() * constants.CLICKS_PER_DEGREE); //total rotation of right module

        rotateL %= constants.CLICKS_PER_PURPLE_REV;
        rotateR %= constants.CLICKS_PER_PURPLE_REV;

        int topLTarget = (int)((robot.topL.getCurrentPosition() - rotateL));
        int botLTarget = (int)((robot.botL.getCurrentPosition() - rotateL));
        int topRTarget = (int)((robot.topR.getCurrentPosition() - rotateR));
        int botRTarget = (int)((robot.botR.getCurrentPosition() - rotateR));


        if (!isResetCycle){
            isResetCycle = true;

            robot.topL.setTargetPosition(topLTarget);
            robot.botL.setTargetPosition(botLTarget);
            robot.topR.setTargetPosition(topRTarget);
            robot.botR.setTargetPosition(botRTarget);

            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else{
//            power = accelerator.update(1.0);
            power = 1;
            robot.botL.setPower(power);
            robot.topL.setPower(power);
            robot.botR.setPower(power);
            robot.topR.setPower(power);
            globalPosSystem.setPoles(true, true);
        }
    }

    ElapsedTime stopTimer = new ElapsedTime();
    private void updateResetAuto(){
        globalPosSystem.calculatePos();
        int rotateL = -(int)(globalPosSystem.getLeftWheelW() * constants.CLICKS_PER_DEGREE); //total rotation of left module
        int rotateR = (int)(globalPosSystem.getRightWheelW() * constants.CLICKS_PER_DEGREE); //total rotation of right module

        rotateL %= constants.CLICKS_PER_PURPLE_REV; //i think there's a problem with this modular function (modding negative numbers is done differently w/ java for some reason?)
        rotateR %= constants.CLICKS_PER_PURPLE_REV;

        int topLTarget = (int)((robot.topL.getCurrentPosition() - rotateL));
        int botLTarget = (int)((robot.botL.getCurrentPosition() - rotateL));
        int topRTarget = (int)((robot.topR.getCurrentPosition() - rotateR));
        int botRTarget = (int)((robot.botR.getCurrentPosition() - rotateR));


        if (!isResetCycle){
            isResetCycle = true;

            robot.topL.setTargetPosition(topLTarget);
            robot.botL.setTargetPosition(botLTarget);
            robot.topR.setTargetPosition(topRTarget);
            robot.botR.setTargetPosition(botRTarget);

            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else{
            powerL = snapLeftWheelPID.update(rotateL * constants.DEGREES_PER_CLICK);
            powerR = snapRightWheelPID.update(rotateR * constants.DEGREES_PER_CLICK);

            robot.botL.setPower(powerL);
            robot.topL.setPower(powerL);
            robot.botR.setPower(powerR);
            robot.topR.setPower(powerR);
            resetDone = (Math.abs(robot.topL.getCurrentPosition() - topLTarget) < 2 &&
                    Math.abs(robot.botL.getCurrentPosition() - botLTarget) < 2 &&
                    Math.abs(robot.topR.getCurrentPosition() - topRTarget) < 2 &&
                    Math.abs(robot.botR.getCurrentPosition() - botRTarget) < 2);

            if (resetDone){
                if (stopTimer.seconds() > 1.5){
                    resetDone = true;
                    globalPosSystem.setPoles(true, true);
                } else resetDone = false;
            } else {
                stopTimer.reset();
                resetDone = false;
            }
        }
    }

    public boolean finishedReset(){
        return resetDone;
    }
}
