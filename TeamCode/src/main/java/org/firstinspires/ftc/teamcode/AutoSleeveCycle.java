package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Color Sleeve Cycle")
public class AutoSleeveCycle extends LinearOpMode {
    static final int MOTOR_TICK_COUNTS = 1120;
    // Name of the Webcam to be set in the config
    private final String webcamName = "Webcam 1";
    // Hardware
    private Hardware hardware;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init hardware
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

        // reset encoders

//        sleep(1000);
//
//        hardware.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        // the cycle

        // set target

        hardware.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setPositionPIDFCoefficients(30);
        hardware.backRightMotor.setPositionPIDFCoefficients(30);
        hardware.frontLeftMotor.setPositionPIDFCoefficients(30);
        hardware.backLeftMotor.setPositionPIDFCoefficients(30);

        hardware.frontRightMotor.setTargetPosition(5000);
        hardware.backRightMotor.setTargetPosition(5000);
        hardware.frontLeftMotor.setTargetPosition(5000);
        hardware.backLeftMotor.setTargetPosition(5000);

        // set speed
        hardware.frontRightMotor.setVelocity(537.7);
        hardware.frontLeftMotor.setVelocity(537.7);
        hardware.backRightMotor.setVelocity(537.7);
        hardware.backLeftMotor.setVelocity(537.7);

        // run
        hardware.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runLift(2000000000);

        while (hardware.frontRightMotor.isBusy() || hardware.frontLeftMotor.isBusy() || hardware.backRightMotor.isBusy() || hardware.backLeftMotor.isBusy()) {
            telemetry.addData("Path", "Moving");

            telemetry.addData("fr ticks", hardware.frontRightMotor.getCurrentPosition());
            telemetry.addData("fl ticks", hardware.frontLeftMotor.getCurrentPosition());
            telemetry.addData("br ticks", hardware.backRightMotor.getCurrentPosition());
            telemetry.addData("bl ticks", hardware.backLeftMotor.getCurrentPosition());
            telemetry.update();
        }

        // stop motor
        hardware.frontRightMotor.setPower(0);
        hardware.frontLeftMotor.setPower(0);
        hardware.backRightMotor.setPower(0);
        hardware.backLeftMotor.setPower(0);

        // the parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
        }
    }
}