package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

@SuppressWarnings("unused")
public abstract class AutoSleeveCycle extends LinearOpMode {
    // PID
    public static double speed = 1200; // Arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0); // PID coefficients that need to be tuned probably through FTC dashboard
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); // PID gains which we will define later in the process
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); // Timer
    // Constants
    final double CLAW_OPEN = -0.1;
    final double CLAW_CLOSE = 0.4;
    final double CLAW_ROTATE_UP = 0.73;
    final double CLAW_ROTATE_DOWN = 0.1;
    final double ARM_PIVOT_SPEED = 0.85;
    // Name of the Webcam to be set in the config
    public String webcamName = "Webcam 1";
    // Hardware
    public Hardware hardware;
    public SleeveDetection sleeveDetection;
    public OpenCvCamera camera;

    public final void strafeLeft(int targetPosition) {
        hardware.frontRightMotor.setTargetPosition(targetPosition);
        hardware.backRightMotor.setTargetPosition(-targetPosition);
        hardware.frontLeftMotor.setTargetPosition(-targetPosition);
        hardware.backLeftMotor.setTargetPosition(targetPosition);
    }

    public final void strafeRight(int targetPosition) {
        hardware.frontRightMotor.setTargetPosition(-targetPosition);
        hardware.backRightMotor.setTargetPosition(targetPosition);
        hardware.frontLeftMotor.setTargetPosition(targetPosition);
        hardware.backLeftMotor.setTargetPosition(-targetPosition);
    }

    public final void rotateRight(int targetPosition) {
        hardware.frontRightMotor.setTargetPosition(-targetPosition);
        hardware.backRightMotor.setTargetPosition(-targetPosition);
        hardware.frontLeftMotor.setTargetPosition(targetPosition);
        hardware.backLeftMotor.setTargetPosition(targetPosition);
    }

    public final void rotateLeft(int targetPosition) {
        hardware.frontRightMotor.setTargetPosition(targetPosition);
        hardware.backRightMotor.setTargetPosition(targetPosition);
        hardware.frontLeftMotor.setTargetPosition(-targetPosition);
        hardware.backLeftMotor.setTargetPosition(-targetPosition);
    }

    public final void setAllWheelMotorPower(double power) {
        hardware.frontRightMotor.setPower(power);
        hardware.frontLeftMotor.setPower(power);
        hardware.backRightMotor.setPower(power);
        hardware.backLeftMotor.setPower(power);
    }

    public final void setAllWheelMotorMode(DcMotor.RunMode mode) {
        hardware.frontRightMotor.setMode(mode);
        hardware.frontLeftMotor.setMode(mode);
        hardware.backRightMotor.setMode(mode);
        hardware.backLeftMotor.setMode(mode);
    }

    public final void setAllWheelMotorTargetPosition(int position) {
        hardware.frontRightMotor.setTargetPosition(position);
        hardware.backRightMotor.setTargetPosition(position);
        hardware.frontLeftMotor.setTargetPosition(position);
        hardware.backLeftMotor.setTargetPosition(position);
        setAllWheelMotorTargetPositionTolerance(10);
    }

    public final void setAllWheelMotorTargetPositionTolerance(int tolerance) {
        // Set motor tolerance on wheels (allows less precision but less pausing/thinking)
        hardware.frontRightMotor.setTargetPositionTolerance(tolerance);
        hardware.backRightMotor.setTargetPositionTolerance(tolerance);
        hardware.frontLeftMotor.setTargetPositionTolerance(tolerance);
        hardware.backLeftMotor.setTargetPositionTolerance(tolerance);
    }

    public final void trackAllWheelCurrentPositionTelemetry() {
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
