package org.firstinspires.ftc.teamcode.swerve.teleop;

public class TrackJoystick {
    int joystickCount = 0;
    double currentJoystickL = 0;
    double prevJoystickL = 0;

    public TrackJoystick(){

    }

    public void trackJoystickL(double lx, double ly){
        joystickCount++;
        if(joystickCount > 3){
            joystickCount = 0;
            prevJoystickL = currentJoystickL;
            currentJoystickL = Math.toDegrees(Math.atan2(lx, ly));
        }
    }

    public double getCurrentJoystickL(){
        return currentJoystickL;
    }

    public double getPrevJoystickL(){
        return prevJoystickL;
    }
}
