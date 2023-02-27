package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings("unused")
@Autonomous(name = "High Right Auto Cycle w/ Park")
public class HighRightAutoSleeveCycle extends AutoSleeveCycle {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        trackTelemetryWhileNotIsStarted();
        waitForStart();

        // hardware.armMotor.setPositionPIDFCoefficients(20);
        // hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Store detected parking position
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        // Pick up preloaded cone and adjust arm position
        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(500);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPosition(675);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
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
        setAllWheelMotorPower(0.8);
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
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Strafe right
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeRight(525);
        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Go forward/line up with junction
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(160);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
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
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Move backwards in parking
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(-225);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // The parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
            // Strafe left
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(750);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
            // Strafe left slightly
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(450);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
            // Strafe right
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeRight(1700);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        }
    }
}
