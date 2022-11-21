package org.firstinspires.ftc.teamcode.swerve.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrackJoystick {
    ElapsedTime timer = new ElapsedTime();
    double prevTime = 0;
    double gap = 100;

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
}
