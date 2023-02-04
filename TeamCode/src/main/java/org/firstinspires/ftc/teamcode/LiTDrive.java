package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LiT Drive Program 2022", group = "Linear Opmode")

public class LiTDrive extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    TouchSensor touchSensor;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo clawServo = null;
    private Servo twistServo = null;
    private DcMotor elevatorMotor = null;
    private DcMotor armMotor = null;

    boolean clawToggle = false;
    boolean rotateToggle = false;

    final double CLAW_OPEN = 0.3;
    final double CLAW_CLOSE = 0;
    final double CLAW_ROTATE_UP = 0.73;
    final double CLAW_ROTATE_DOWN = 0;
    
    double armPivotSpeed = 0.75;


    public void runOpMode() {

        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        clawServo = hardwareMap.get(Servo.class, "Servo");
        twistServo = hardwareMap.get(Servo.class, "twist");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor"); //1
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //0
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        // wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Toggle(currentGamepad2, previousGamepad2);
            Claw();
            RotateClaw();
            Drive();
            Elevator();
            ArmPivot();
        }

    }

    public void Toggle(Gamepad currentGamepad2, Gamepad previousGamepad2){
        try {
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
        } catch (Exception e) {
            // e.printStackTrace();
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            clawToggle = !clawToggle;
        }
        if (currentGamepad2.b && !previousGamepad2.b) {
            rotateToggle = !rotateToggle;
        }
    }

    public void Claw() {
        if (clawToggle) {
            clawServo.setPosition(CLAW_CLOSE);
        }
        else {
            clawServo.setPosition(CLAW_OPEN);
        }

        if (touchSensor.isPressed()) {
            clawServo.setPosition(CLAW_CLOSE);
        }
    }

    public void RotateClaw() {
        if (rotateToggle) {
            twistServo.setPosition(CLAW_ROTATE_UP);
        }
        else {
            twistServo.setPosition(CLAW_ROTATE_DOWN);
        }
    }

    public  void Drive() {
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

        double QJSpeed = 1.75;
        double sniperPercent = 0.25;

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
    }

    public void Elevator() {
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
    }

    public void ArmPivot() {
        double armPower = -gamepad2.left_stick_y;
        armMotor.setPower(armPower * armPivotSpeed);
    }

}
