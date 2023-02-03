package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LiT Drive Program 2022", group = "Linear Opmode")

public class LiTDrive extends LinearOpMode {

    double clawPosition = 0.5;
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo clawServo = null;
    private Servo twistServo = null;
    private DcMotor elevatorMotor = null;
    private DcMotor armMotor = null;
    TouchSensor touchSensor;
    //private DcMotor duckMotor = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        clawServo = hardwareMap.get(Servo.class, "Servo");
        twistServo = hardwareMap.get(Servo.class, "twist");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor"); //1
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //0
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            // do not change these values
            double clawOpen = 0.3;
            double clawClose = 0;
            double clawRotateUp = 0.73;
            double clawRotateDown = 0;

            // open/close claw
            if (gamepad2.a) {
                clawServo.setPosition(clawClose);
            }
            else if (gamepad2.right_trigger > 0) {
                clawServo.setPosition(clawOpen);
            }

            // rotate claw
            if (gamepad2.left_bumper) {
                twistServo.setPosition(clawRotateUp);
            }
            if (gamepad2.right_bumper) {
                twistServo.setPosition(clawRotateDown);
            }

            // warning: suspicous looking arm code
            double k = 0;
            if (gamepad2.left_stick_y > 0) {
                k = 1;
            }
            else {
                k = 1;
            }
            double arm = gamepad2.left_stick_y;
            double sniperPercent = 0.35;
            if (gamepad2.right_bumper) {
                k = 1.5;
                armMotor.setPower(arm*(-k)*sniperPercent);
                k = 0;
            }
            else {
                armMotor.setPower(arm * (-k));
            }

            // move the slides
            // down
            if (gamepad2.right_stick_y > 0.25) {
                elevatorMotor.setPower(-0.45);
            }
            // up
            else if (gamepad2.right_stick_y < -0.25) {
                elevatorMotor.setPower(0.45);
            }
            else {
                elevatorMotor.setPower(0);
            }

            // cycle mode auto close
            if (gamepad2.left_trigger > 0) {
                if (touchSensor.isPressed()) {
                    clawServo.setPosition(clawClose);
                }
            }

            // DRIVE CODE

            //MECANUM
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            // STRAFING
            double FL = Range.clip(drive + strafe + turn, -0.5, 0.5);
            double FR = Range.clip(drive - strafe + turn, -0.5, 0.5);
            double BL = Range.clip(drive - strafe - turn, -0.5, 0.5);
            double BR = Range.clip(drive + strafe - turn, -0.5, 0.5);

            // SET SPEED BASED ON DRIVER
            // double speed = 0.78; OLD SPEED
            double QJSpeed = 1.75;
            // DRIVE SYSTEM
            // SNIPER MODE
            if (gamepad1.left_trigger > 0) {
                frontLeftMotor.setPower(FL * QJSpeed * sniperPercent);
                frontRightMotor.setPower(FR * QJSpeed * sniperPercent);
                backLeftMotor.setPower(BL * QJSpeed * sniperPercent);
                backRightMotor.setPower(BR * QJSpeed * sniperPercent);
            }

            // BRAKES
            else if (gamepad1.right_trigger > 0) {
                frontLeftMotor.setPower(FL * 0);
                frontRightMotor.setPower(FR * 0);
                backLeftMotor.setPower(BL * 0);
                backRightMotor.setPower(BR * 0);

            }

            // NORMAL DRIVE
            else {
                frontLeftMotor.setPower(FL * QJSpeed);
                frontRightMotor.setPower(FR * QJSpeed);
                backLeftMotor.setPower(BL * QJSpeed);
                backRightMotor.setPower(BR * QJSpeed);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }}
