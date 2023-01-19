package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;

public class Reset {
    HardwareDrive robot;
    GlobalPosSystem globalPosSystem;
    Constants constants = new Constants();
    ElapsedTime gapTime = new ElapsedTime();
    Accelerator accelerator = new Accelerator();
    double power = 0;
    int waitForMS = 400;
    double prevTime=0;
    double currentTime=0;

    boolean STOP_RESET_L = false;
    boolean STOP_RESET_R = false;
    boolean isResetCycle = false;

    public Reset(HardwareDrive r, GlobalPosSystem gps){
        robot = r;
        globalPosSystem=gps;
        gapTime.reset();
    }

    public void reset(boolean shouldReset){
        if (shouldReset){
            if(gapTime.milliseconds()-prevTime>waitForMS){
                updateReset();
            }else{
                power = accelerator.update(0);
                robot.botL.setPower(power);
                robot.topL.setPower(power);
                robot.botR.setPower(power);
                robot.topR.setPower(power);
            }

        } else{
            STOP_RESET_L = false;
            STOP_RESET_R = false;
            isResetCycle = false;
            gapTime.reset();
            prevTime=gapTime.milliseconds();
        }
    }

    private void updateReset(){
        globalPosSystem.calculatePos();
        int rotateL = (int)(globalPosSystem.getLeftWheelW() * constants.CLICKS_PER_DEGREE); //total rotation of left module
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

            power = accelerator.update(0.7);
            robot.botL.setPower(power);
            robot.topL.setPower(power);
            robot.botR.setPower(power);
            robot.topR.setPower(power);
        }

        else{
            if (robot.topL.getCurrentPosition() == topLTarget && robot.botL.getCurrentPosition() == botLTarget){
                robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                power = accelerator.update(0);
                robot.botL.setPower(power);
                robot.topL.setPower(power);

                STOP_RESET_L = true;
            } else if (!STOP_RESET_L){
                power = accelerator.update(0.7);
                robot.botL.setPower(power);
                robot.topL.setPower(power);
            }

            if (robot.topR.getCurrentPosition() == topRTarget && robot.botR.getCurrentPosition() == botRTarget){
                robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                power = accelerator.update(0);
                robot.botR.setPower(power);
                robot.topR.setPower(power);

                STOP_RESET_R = true;
            } else if (!STOP_RESET_R){
                power = accelerator.update(0.7);
                robot.botR.setPower(power);
                robot.topR.setPower(power);
            }

            if (finishedReset()) {
                robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                globalPosSystem.hardResetGPS();
            }
        }
    }

    public boolean finishedReset(){
        return (STOP_RESET_L && STOP_RESET_R);
    }
}
