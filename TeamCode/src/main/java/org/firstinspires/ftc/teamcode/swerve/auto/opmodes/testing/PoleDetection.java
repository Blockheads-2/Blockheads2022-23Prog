package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mecanum.common.Constants;
import org.firstinspires.ftc.teamcode.swerve.auto.cv.CameraMaster;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Pole Detection Test", group = "Drive")
public class PoleDetection extends LinearOpMode {

    OpenCvCamera phoneCam;
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        CameraMaster detector = new CameraMaster();
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)

        }


        telemetry.addData("Pole Position X", detector.getJunctionPoint().x);
        telemetry.addData("Pole Position Y", detector.getJunctionPoint().y);
        telemetry.addData("Number of Contours:", detector.getNumOfContours());
        telemetry.addData("Distance to Pole", detector.getJunctionDistance());
        telemetry.update();

        waitForStart();

    }
}