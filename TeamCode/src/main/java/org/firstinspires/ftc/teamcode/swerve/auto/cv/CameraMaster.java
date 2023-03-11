package org.firstinspires.ftc.teamcode.swerve.auto.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CameraMaster extends OpenCvPipeline {

    Scalar darkestJunctions = new Scalar(15, 100, 100);
    Scalar lightestJunctions = new Scalar(60, 255, 255);
    Constants constants = new Constants();

    Mat rawHSV = new Mat();
    Mat blurredHSV = new Mat();
    Mat thresholded = new Mat();

    MatOfPoint biggestContour;
    int totalContours = 0;
    Point junctionPoint = new Point();
    double junctionDistanceAttr = 0;
    int numOfContours=  0;

    //custom coordinate
    double alpha = 0;
    double beta = 0;

    RotatedRect box = new RotatedRect();

    List<MatOfPoint> contours = new ArrayList<>();

    Telemetry telemetry;

    public CameraMaster(){

    }

    public CameraMaster(Telemetry t){
        this.telemetry = t;
    }

    public Mat processFrame(Mat input) {
        // Convert image to HSV
        Imgproc.cvtColor(input, rawHSV, Imgproc.COLOR_RGB2HSV);

        // Blur image to lessen noise
        Imgproc.GaussianBlur(rawHSV, blurredHSV, new Size(15, 15), 0); // make box bigger

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Core.inRange(blurredHSV, darkestJunctions, lightestJunctions, thresholded);

        // Find contours
//        List<MatOfPoint> contours = new ArrayList<>();
        contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Get distance and centroid of biggest junction
        if (!contours.isEmpty()) {
            biggestContour = contours.get(contours.size()-1);


            for (int i = contours.size()-1; i >= 0; i--){
                if (Imgproc.contourArea(contours.get(i)) <= 5000){ // last tested was 5000
                    contours.remove(i);
                }
            }

            for (MatOfPoint contour : contours){
                if (Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour)){
                    biggestContour = contour;
                }
            }
//            for (int i = contours.size()-1; i >= 0; i--){
//                for (int j = i-1; j >= 0; j--){
//                    if (contours.size() > 1 && Imgproc.contourArea(contours.get(i)) < Imgproc.contourArea(contours.get(j))) {
//                        contours.remove(i);
//                        break;
//                    }
//                }
//            }

            // Finds center / most concentrated part
            Moments moments = Imgproc.moments(biggestContour); //an image moment is a certain particular weighted average (moment) of the image pixels' intensities, or a function of such moments

            //Assign attributes
            totalContours = contours.size();
//            junctionDistanceAttr = 240000/Imgproc.contourArea(biggestContour);
            this.junctionPoint = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            alpha = junctionPoint.x - (constants.CAMERA_WIDTH / 2.0);
            beta = (constants.CAMERA_HEIGHT - junctionPoint.y);
//            Imgproc.boxPoints(box, biggestContour);

            SpitTelemetry();
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 3);

        List<MatOfPoint> biggestContourList = new ArrayList<>();
        biggestContourList.add(biggestContour);

        Imgproc.drawContours(input, biggestContourList, -1, new Scalar(255,0,0), 3);


        return input;
    }

    public List<MatOfPoint> getContours(){
        return contours;
    }

    public Point getJunctionPoint() {
        return junctionPoint;
    }
    public int getNumOfContours() {return totalContours;}
//    public double getJunctionDistance() {
//        return junctionDistanceAttr; // this is in inches
//    }
    public double getAngle(){return Math.toDegrees(Math.atan2(alpha, beta));}

    public MatOfPoint getBiggestContour(){
        return biggestContour;
    }
    public RotatedRect getBox(){
        return box;
    }
    public double getAlpha(){
        return alpha;
    }

    public double getBeta(){
        return beta;
    }

    public void SpitTelemetry(){
        telemetry.addData("Pole Position X", getJunctionPoint().x);
        telemetry.addData("Pole Position Y", getJunctionPoint().y);
        telemetry.addData("Number of Contours:", getNumOfContours());
        telemetry.addData("Alpha", getAlpha());
        telemetry.addData("Beta", getBeta());
        telemetry.addData("Angle", getAngle());

        telemetry.update();
    }
}