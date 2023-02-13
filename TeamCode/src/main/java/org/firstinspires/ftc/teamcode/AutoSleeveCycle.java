package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Color Sleeve Cycle")
public class AutoSleeveCycle extends LinearOpMode {
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

        // The cycle
        // Go forward
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorTargetPosition(2300);
        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Rotate right
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(-850);
        hardware.backRightMotor.setTargetPosition(-850);
        hardware.frontLeftMotor.setTargetPosition(850);
        hardware.backLeftMotor.setTargetPosition(850);

        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Strafe left
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(400);
        hardware.backRightMotor.setTargetPosition(-400);
        hardware.frontLeftMotor.setTargetPosition(-400);
        hardware.backLeftMotor.setTargetPosition(400);

        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Go forward/line up with junction
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorTargetPosition(150);
        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // The parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
        }
    }

    public void setAllMotorPower(double power) {
        hardware.frontRightMotor.setPower(power);
        hardware.frontLeftMotor.setPower(power);
        hardware.backRightMotor.setPower(power);
        hardware.backLeftMotor.setPower(power);
    }

    public void setAllMotorMode(DcMotor.RunMode mode) {
        hardware.frontRightMotor.setMode(mode);
        hardware.frontLeftMotor.setMode(mode);
        hardware.backRightMotor.setMode(mode);
        hardware.backLeftMotor.setMode(mode);
    }

    public void setAllMotorTargetPosition(int position) {
        hardware.frontRightMotor.setTargetPosition(position);
        hardware.backRightMotor.setTargetPosition(position);
        hardware.frontLeftMotor.setTargetPosition(position);
        hardware.backLeftMotor.setTargetPosition(position);
    }

    public void trackCurrentPositionTelemetry() {
        while (hardware.frontRightMotor.isBusy() || hardware.frontLeftMotor.isBusy() || hardware.backRightMotor.isBusy() || hardware.backLeftMotor.isBusy()) {
            telemetry.addData("Path", "Moving");
            telemetry.addData("fr ticks", hardware.frontRightMotor.getCurrentPosition());
            telemetry.addData("fl ticks", hardware.frontLeftMotor.getCurrentPosition());
            telemetry.addData("br ticks", hardware.backRightMotor.getCurrentPosition());
            telemetry.addData("bl ticks", hardware.backLeftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
