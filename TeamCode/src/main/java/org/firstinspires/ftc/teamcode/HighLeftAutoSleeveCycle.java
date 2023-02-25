package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "High Left Auto Cycle w/ Park")
public class HighLeftAutoSleeveCycle extends LinearOpMode {
    public static double speed = 1200; // Arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); // PID coefficients that need to be tuned probably through FTC dashboard
    final double CLAW_OPEN = -0.1;
    final double CLAW_CLOSE = 0.4;
    final double CLAW_ROTATE_UP = 0.73;
    final double CLAW_ROTATE_DOWN = 0.1;
    // Name of the Webcam to be set in the config
    private final String webcamName = "Webcam 1";
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); // PID gains which we will define later in the process
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // Timer
    // Hardware
    private Hardware hardware;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private Servo clawServo;
    private Servo twistServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx armMotor;

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

        clawServo = hardwareMap.get(Servo.class, "Servo");
        twistServo = hardwareMap.get(Servo.class, "twist");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("Path", "Moving");
            telemetry.addData("fr ticks", hardware.frontRightMotor.getCurrentPosition());
            telemetry.addData("fl ticks", hardware.frontLeftMotor.getCurrentPosition());
            telemetry.addData("br ticks", hardware.backRightMotor.getCurrentPosition());
            telemetry.addData("bl ticks", hardware.backLeftMotor.getCurrentPosition());
            telemetry.addData("arm ticks", hardware.armMotor.getCurrentPosition());
            telemetry.addData("elevator ticks", elevatorMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        // hardware.armMotor.setPositionPIDFCoefficients(20);
        // hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The cycle
        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(500);
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(675);
        hardware.armMotor.setPower(0.5);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hardware.armMotor.isBusy()) {
            twistServo.setPosition(CLAW_ROTATE_UP);
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
            elevatorMotor.setPower(0.7);
            sleep(1500);
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rotate right
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(-850);
        hardware.backRightMotor.setTargetPosition(-850);
        hardware.frontLeftMotor.setTargetPosition(850);
        hardware.backLeftMotor.setTargetPosition(850);

        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Strafe left
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(375);
        hardware.backRightMotor.setTargetPosition(-375);
        hardware.frontLeftMotor.setTargetPosition(-375);
        hardware.backLeftMotor.setTargetPosition(375);

        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Go forward/line up with junction
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(150);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetry();
        setAllWheelMotorPower(0);

        // Arm cycle starts
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(-300);
        hardware.armMotor.setPower(0.75);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }
        hardware.armMotor.setPower(0);

        hardware.clawServo.setPosition(CLAW_OPEN);
        sleep(1000);

        // move down for the first cycle
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(1800);
        hardware.armMotor.setPower(0.4);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy()) {
            twistServo.setPosition(CLAW_ROTATE_DOWN);
        }
        clawServo.setPosition(CLAW_CLOSE);
        sleep(1500);
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(-1800);
        hardware.armMotor.setPower(0.4);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy()) {
            sleep(1000);
            twistServo.setPosition(CLAW_ROTATE_UP);
        }
        clawServo.setPosition(CLAW_OPEN);
        sleep(1500);
        // finish cycle
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(500);
        hardware.armMotor.setPower(0.75);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy()) {
            clawServo.setPosition(CLAW_OPEN);
        }

        elevatorMotor.setPower(-0.7);
        sleep(1500);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rotate right to park
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRightMotor.setTargetPosition(1200);
        hardware.backRightMotor.setTargetPosition(1200);
        hardware.frontLeftMotor.setTargetPosition(-1200);
        hardware.backLeftMotor.setTargetPosition(-1200);
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

            hardware.frontRightMotor.setTargetPosition(1700);
            hardware.backRightMotor.setTargetPosition(-1700);
            hardware.frontLeftMotor.setTargetPosition(-1700);
            hardware.backLeftMotor.setTargetPosition(1700);

            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
            // Strafe left
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            hardware.frontRightMotor.setTargetPosition(500);
            hardware.backRightMotor.setTargetPosition(-500);
            hardware.frontLeftMotor.setTargetPosition(-500);
            hardware.backLeftMotor.setTargetPosition(500);

            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
            // Strafe right
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            hardware.frontRightMotor.setTargetPosition(-750);
            hardware.backRightMotor.setTargetPosition(750);
            hardware.frontLeftMotor.setTargetPosition(750);
            hardware.backLeftMotor.setTargetPosition(-750);

            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetry();
            setAllWheelMotorPower(0);
        }
    }

    public void setAllWheelMotorPower(double power) {
        hardware.frontRightMotor.setPower(power);
        hardware.frontLeftMotor.setPower(power);
        hardware.backRightMotor.setPower(power);
        hardware.backLeftMotor.setPower(power);
    }

    public void setAllWheelMotorMode(DcMotor.RunMode mode) {
        hardware.frontRightMotor.setMode(mode);
        hardware.frontLeftMotor.setMode(mode);
        hardware.backRightMotor.setMode(mode);
        hardware.backLeftMotor.setMode(mode);
    }

    public void setAllWheelMotorTargetPosition(int position) {
        hardware.frontRightMotor.setTargetPosition(position);
        hardware.backRightMotor.setTargetPosition(position);
        hardware.frontLeftMotor.setTargetPosition(position);
        hardware.backLeftMotor.setTargetPosition(position);
        // Set motor tolerance on wheels (allows less precision)
        hardware.frontRightMotor.setTargetPositionTolerance(10);
        hardware.backRightMotor.setTargetPositionTolerance(10);
        hardware.frontLeftMotor.setTargetPositionTolerance(10);
        hardware.backLeftMotor.setTargetPositionTolerance(10);
    }

    public void trackAllWheelCurrentPositionTelemetry() {
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
