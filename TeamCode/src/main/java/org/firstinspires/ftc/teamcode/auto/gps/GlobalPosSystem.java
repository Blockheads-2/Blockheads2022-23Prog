package org.firstinspires.ftc.teamcode.auto.gps;

public class GlobalPosSystem {

    private double[] position = new double[3];

    public GlobalPosSystem(){
        for (int i = 0; i < 3; i++){
            position[i] = 0;
        }

    }

    public void calculatePos(){
        int translationalClicks;
        int rotationalClicks;
    }

    public void update(double x, double y, double theta){
        //update
        position[0] = x;
        position[1] = y;
        position[2] = theta;
    }

    public double[] getPosition(){
        return position;
    }


}
