package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.swerve.auto.cv.CameraMaster;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
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
        CameraMaster detector = new CameraMaster(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(constants.CAMERA_WIDTH, constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(phoneCam, 5);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Pole Position X", detector.getJunctionPoint().x);
            telemetry.addData("Pole Position Y", detector.getJunctionPoint().y);
            telemetry.addData("Number of Contours:", detector.getNumOfContours());
//            telemetry.addData("Distance to Pole", detector.getJunctionDistance());
            telemetry.addData("Angle", detector.getAngle());
//            telemetry.addData("center.x", detector.getBox().center.x);
//            telemetry.addData("center.y", detector.getBox().center.y);
//            telemetry.addData("size.width", detector.getBox().size.width);
//            telemetry.addData("size.height", detector.getBox().size.height);
//            telemetry.addData("angle", detector.getBox().angle);
            telemetry.addData("Alpha", detector.getAlpha());
            telemetry.addData("Beta", detector.getBeta());

            telemetry.update();
        }


    }
}