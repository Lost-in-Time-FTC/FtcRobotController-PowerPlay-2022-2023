package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Color Sleeve Parking")
public class AutoSleevePark extends LinearOpMode {

    // Hardware things
    private Hardware hardware;

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private final String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {

        // Init hardware things
        hardware = new Hardware(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        SleeveDetection.ParkingPosition pos = sleeveDetection.getPosition();
        hardware.driveForwardTime(0.5, 900);
        if (pos == SleeveDetection.ParkingPosition.LEFT) {
            hardware.driveLeftTime(0.7, 1500);
        } else if (pos == SleeveDetection.ParkingPosition.RIGHT) {
            hardware.driveRightTime(0.5, 1300);
        }

    }
}
