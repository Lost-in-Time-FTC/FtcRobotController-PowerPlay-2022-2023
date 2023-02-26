package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@SuppressWarnings("unused")
@Autonomous(name = "High Right Auto Cycle w/ Park")
public class HighRightAutoSleeveCycle extends AutoSleeveCycle {
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
            telemetry.addData("Parking Position: ", sleeveDetection.getPosition());
            telemetry.addData("Path", "Moving");
            telemetry.addData("fr ticks", hardware.frontRightMotor.getCurrentPosition());
            telemetry.addData("fl ticks", hardware.frontLeftMotor.getCurrentPosition());
            telemetry.addData("br ticks", hardware.backRightMotor.getCurrentPosition());
            telemetry.addData("bl ticks", hardware.backLeftMotor.getCurrentPosition());
            telemetry.addData("arm ticks", hardware.armMotor.getCurrentPosition());
            telemetry.addData("elevator ticks", hardware.elevatorMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        // hardware.armMotor.setPositionPIDFCoefficients(20);
        // hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        // The cycle
        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(500);
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(675);
        hardware.armMotor.setPower(0.5);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hardware.armMotor.isBusy()) {
            hardware.twistServo.setPosition(CLAW_ROTATE_UP);
            telemetry.addData("Arm", "Moving");
            telemetry.addData("arm ticks", hardware.armMotor.getCurrentPosition());
            telemetry.update();
        }
        hardware.armMotor.setPower(0);

        // Go forward
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(2300);
        setAllWheelMotorPower(0.6);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hardware.frontRightMotor.isBusy() || hardware.frontLeftMotor.isBusy() || hardware.backRightMotor.isBusy() || hardware.backLeftMotor.isBusy()) {
            hardware.elevatorMotor.setPower(0.7);
            sleep(1500);
            hardware.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        hardware.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rotate left
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateLeft(900);
        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Strafe right
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeRight(525);
        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Go forward/line up with junction
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(160);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Arm cycle starts
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(-300);
        hardware.armMotor.setPower(0.75);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hardware.armMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }
        hardware.armMotor.setPower(0);

        hardware.clawServo.setPosition(CLAW_OPEN);
        sleep(1000);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(500);
        hardware.armMotor.setPower(0.75);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hardware.armMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }
        hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rotate right to park
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateRight(1200);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Move backwards in parking
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(-225);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // The parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
            // Strafe left
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(750);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
            // Strafe left slightly
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(450);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
            // Strafe right
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeRight(1700);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        }
    }
}
