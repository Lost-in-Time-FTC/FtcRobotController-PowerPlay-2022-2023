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

@Autonomous(name = "Color Sleeve Cycle")
public class AutoSleeveCycle extends LinearOpMode {
    public static double speed = 1200; //arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned probably through FTC dashboard
    final double CLAW_OPEN = 0;
    final double CLAW_CLOSE = 0.4;
    final double CLAW_ROTATE_UP = 0.73;
    final double CLAW_ROTATE_DOWN = 0.1;
    // Name of the Webcam to be set in the config
    private final String webcamName = "Webcam 1";
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer
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
        //hardware.armMotor.setPositionPIDFCoefficients(20);
        //hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        setAllMotorPower(0);

        // Go forward
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorTargetPosition(2300);
        setAllMotorPower(0.5);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        // Rotate right
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(-900);
        hardware.backRightMotor.setTargetPosition(-900);
        hardware.frontLeftMotor.setTargetPosition(900);
        hardware.backLeftMotor.setTargetPosition(900);

        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Strafe left
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontRightMotor.setTargetPosition(450);
        hardware.backRightMotor.setTargetPosition(-450);
        hardware.frontLeftMotor.setTargetPosition(-450);
        hardware.backLeftMotor.setTargetPosition(450);

        setAllMotorPower(0.25);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Go forward/line up with junction
        setAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorTargetPosition(180);
        setAllMotorPower(0.5);
        setAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackCurrentPositionTelemetry();
        setAllMotorPower(0);

        // Arm cycle starts
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setTargetPosition(1250);
        // SINCE THE ELEVATOR CANNOT REACH THIS POSITION, IT KEEPS TRYING THEN RUNS OUT OF TIME SO YOU HAVE TO KEEP THIS SLIGHLY LESS THAN THE MAX EXTENSION FIX ON SATURDAY-johan
        elevatorMotor.setPower(0.45);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (elevatorMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }
        elevatorMotor.setPower(0);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(-250);
        hardware.armMotor.setPower(0.5);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }
        hardware.armMotor.setPower(0);

        hardware.clawServo.setPosition(CLAW_OPEN);

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

    public void moveArm(int target) {
//        double kp = 5.0;
//        int threshold = 500;
//        int error = target - hardware.armMotor.getCurrentPosition();
//
//        telemetry.addData("error", error);
//        telemetry.update();
//
//        while (Math.abs(error) > threshold) {
//            hardware.armMotor.setPower(kp * error);
//            telemetry.addData("did it run", "it ran");
//            telemetry.update();
//        }
//        hardware.armMotor.setPower();
    }
}
