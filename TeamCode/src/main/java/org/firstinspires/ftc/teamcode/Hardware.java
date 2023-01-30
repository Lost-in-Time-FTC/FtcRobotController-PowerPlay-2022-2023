package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware {
    double clawPosition = 0.5;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo clawServo = null;
    private final Servo twistServo = null;
    private DcMotor elevatorMotor = null;
    private DcMotor armMotor = null;
    //private DcMotor duckMotor = null;

    public Hardware(HardwareMap hardwareMap) {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        clawServo = hardwareMap.get(Servo.class, "Servo"); //
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor"); //1
        armMotor = hardwareMap.get(DcMotor.class, "armMotor"); //0


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveForwardTime(double power, int time) throws InterruptedException {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        Thread.sleep(time);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void driveBackwardTime(double power, int time) throws InterruptedException {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        Thread.sleep(time);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void driveRightTime(double power, int time) throws InterruptedException {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        Thread.sleep(time);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void driveLeftTime(double power, int time) throws InterruptedException {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        Thread.sleep(time);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
}
