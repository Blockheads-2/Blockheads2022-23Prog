package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.swerve.auto.cv.CameraMaster;
import org.firstinspires.ftc.teamcode.swerve.auto.cv.CameraMasterJR;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Stream Camera Test", group = "Drive")
public class StreamCamera extends LinearOpMode {

    OpenCvCamera phoneCam;
    Constants constants = new Constants();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        CameraMasterJR detector = new CameraMasterJR();
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(constants.WIDTH, constants.HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
//                FtcDashboard.getInstance().startCameraStream(phoneCam, 5);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)

        }

        waitForStart();

        while (opModeIsActive()){

        }


    }
}