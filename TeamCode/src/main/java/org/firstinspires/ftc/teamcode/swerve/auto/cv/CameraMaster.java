package org.firstinspires.ftc.teamcode.swerve.auto.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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
    Mat rawHSV = new Mat();
    Mat blurredHSV = new Mat();
    Mat thresholded = new Mat();
    int junctionNumAttr = 0;
    Point junctionPointAttr = new Point();
    double junctionDistanceAttr = 0;
    int numOfContours=  0;

    List<MatOfPoint> contours = new ArrayList<>();

    public Mat processFrame(Mat input) {
        // Convert image to HSV
        Imgproc.cvtColor(input, rawHSV, Imgproc.COLOR_RGB2HSV);

        // Blur image to lessen noise
        Imgproc.GaussianBlur(rawHSV, blurredHSV, new Size(15, 15), 0); // increase blur?

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Core.inRange(blurredHSV, darkestJunctions, lightestJunctions, thresholded);

        // Find contours
//        List<MatOfPoint> contours = new ArrayList<>();
        contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Get distance and centroid of biggest junction
        if (!contours.isEmpty()) {
            MatOfPoint biggestContour = contours.get(0);


            for (int i = contours.size()-1; i >= 0; i--){
                if (Imgproc.contourArea(contours.get(i)) <= 7000){ // last tested was 5000
                    contours.remove(i);
                }
            }


            for (MatOfPoint curContour : contours) {
                if (Imgproc.contourArea(curContour) > Imgproc.contourArea(biggestContour)) {
                    biggestContour = curContour;
                }
            }

            // Find centroid
            Moments moments = Imgproc.moments(biggestContour);
            Point junctionPoint = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            //Assign attributes
            junctionNumAttr = contours.size();
            junctionDistanceAttr = 240000/Imgproc.contourArea(biggestContour);
            junctionPointAttr = junctionPoint;
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 3);

        return input;
    }

    public List<MatOfPoint> getContours(){
        return contours;
    }

    public Point getJunctionPoint() {
        return junctionPointAttr;
    }
    public int getNumOfContours() {return junctionNumAttr;}
    public double getJunctionDistance() {
        return junctionDistanceAttr; // this is in inches
    }
    public double getAngle(){return Math.toDegrees(Math.atan2(junctionPointAttr.x, junctionPointAttr.y));}
}