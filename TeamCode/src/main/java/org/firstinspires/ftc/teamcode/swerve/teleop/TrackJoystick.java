package org.firstinspires.ftc.teamcode.swerve.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.stream.IntStream;

public class TrackJoystick {
    ElapsedTime timer = new ElapsedTime();
    double prevTime = 0;
    double gap = 175; //play around with this number!

    double currentJoystickL = 0;
    double prevJoystickL = 0;


    public TrackJoystick(){
        timer.reset();
    }

    public void trackJoystickL(double lx, double ly){
        if(timer.milliseconds() - prevTime > gap){
            prevJoystickL = currentJoystickL;
            prevTime = timer.milliseconds();
        }
        currentJoystickL = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) currentJoystickL = 0;
        else if (lx==0 && ly < 0) currentJoystickL=180;
    }

    public double getCurrentJoystickL(){
        return currentJoystickL;
    }

    public double getPrevJoystickL(){
        return prevJoystickL;
    }

    public double getChange(){
        double curr2 = (currentJoystickL < 0 ? currentJoystickL + 360 : currentJoystickL);
        double prev2 = (prevJoystickL < 0 ? prevJoystickL + 360 : prevJoystickL);

        double turnAmount = currentJoystickL - prevJoystickL;
        double turnAmount2 = curr2 - prev2;

        if (Math.abs(turnAmount) < Math.abs(turnAmount2)){
            return turnAmount;

        } else{
            return turnAmount2;
        }
    }

    public double getAbsoluteChange(){
        double curr2 = (currentJoystickL < 0 ? currentJoystickL + 360 : currentJoystickL);
        double prev2 = (prevJoystickL < 0 ? prevJoystickL + 360 : prevJoystickL);

        double turnAmount = currentJoystickL - prevJoystickL;
        double turnAmount2 = curr2 - prev2;

        if (Math.abs(turnAmount) < Math.abs(turnAmount2)){
            return Math.abs(turnAmount);

        } else{
            return Math.abs(turnAmount2);
        }
    }

    public double getAngle(double x, double y){
        double target = Math.toDegrees(Math.atan2(x, y));
        if (target >= 0) {
            for (int i = 0; i < 2; i++) {
                if (target >= ((i * 90) - 10) && target <= ((i * 90) + 10)) {
                    target = i * 90;
                }
            }
        }
        else if (target < 0){
            for (int i = 0; i > -2; i--) {
                if (target >= ((i * 90) - 10) && target <= ((i * 90) + 10)) {
                    target = i * 90;
                }
            }
        }
        if (x == 0 && y == 0) target = 0;
        else if (x==0 && y < 0) target=180;

        return target;
    }

    public void changeGapTime(int ms){
        gap -= ms;
        if (gap > 500) gap = 500;
        else if (gap < 100) gap = 100;
    }

    public double getGapTime(){
        return gap;
    }
}
