package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings("unused")
@Autonomous(name = "High Left Auto Cycle w/ Park")
public class HighLeftAutoSleeveCycle extends AutoSleeveCycle {
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
        sleep(300);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
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

        // Rotate right
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateRight(850);
        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Strafe left
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeLeft(375);
        setAllWheelMotorPower(0.25);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Go forward/line up with junction
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(150);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Arm cycle starts
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
        hardware.armMotor.setTargetPosition(-400);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            telemetry.addData("moving", "moving");
            telemetry.update();
        }

        hardware.armMotor.setPower(0);

        hardware.clawServo.setPosition(CLAW_WIDE_OPEN);
        sleep(250);
        hardware.clawServo.setPosition(CLAW_OPEN);

        // Move down for the first cycle
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
        hardware.armMotor.setTargetPosition(1825); // Probably correct?
        hardware.armMotor.setPower(1);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            hardware.twistServo.setPosition(CLAW_ROTATE_DOWN);
        }

        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(400);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_HIGH_TOLERANCE);
        hardware.armMotor.setTargetPosition(-1900);
        hardware.armMotor.setPower(1);
        sleep(200);
        hardware.armMotor.setPower(0);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_UP);
        }

        hardware.clawServo.setPosition(CLAW_WIDE_OPEN);
        sleep(250);
        hardware.clawServo.setPosition(CLAW_OPEN);

        // move down for 3rd cone
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(50);
        hardware.armMotor.setTargetPosition(1950);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED_DOWN);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_DOWN);
        }

        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(400);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_HIGH_TOLERANCE);
        hardware.armMotor.setTargetPosition(-1900);
        hardware.armMotor.setPower(1);
        sleep(200);
        hardware.armMotor.setPower(0);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_UP);
        }

        hardware.clawServo.setPosition(CLAW_WIDE_OPEN);
        sleep(250);
        hardware.clawServo.setPosition(CLAW_OPEN);

        // move down for 4th cone
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
        hardware.armMotor.setTargetPosition(2050);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED_DOWN);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_DOWN);
        }

        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(400);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_HIGH_TOLERANCE);
        hardware.armMotor.setTargetPosition(-2000);
        hardware.armMotor.setPower(1);
        sleep(200);
        hardware.armMotor.setPower(0);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_UP);
        }

        hardware.clawServo.setPosition(CLAW_WIDE_OPEN);
        sleep(250);
        hardware.clawServo.setPosition(CLAW_OPEN);

        // move down for 5th cone
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
        hardware.armMotor.setTargetPosition(2150);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED_DOWN);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_DOWN);
        }

        hardware.clawServo.setPosition(CLAW_CLOSE);
        sleep(400);

        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_HIGH_TOLERANCE);
        hardware.armMotor.setTargetPosition(-2050);
        hardware.armMotor.setPower(1);
        sleep(200);
        hardware.armMotor.setPower(0);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            sleep(250);
            hardware.twistServo.setPosition(CLAW_ROTATE_UP);
        }

        hardware.clawServo.setPosition(CLAW_WIDE_OPEN);
        sleep(250);
        hardware.clawServo.setPosition(CLAW_OPEN);

        // Finish cycle
        hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor.setTargetPositionTolerance(ARM_POSITION_LOW_TOLERANCE);
        hardware.armMotor.setTargetPosition(500);
        hardware.armMotor.setPower(ARM_PIVOT_SPEED);
        hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.armMotor.isBusy()) {
            hardware.clawServo.setPosition(CLAW_OPEN);
        }

        hardware.elevatorMotor.setPower(-0.7);
        sleep(1500);
        hardware.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Go forward/line up with junction again
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(40);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Rotate left to park
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateLeft(1100);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // Move backwards in parking
        setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllWheelMotorTargetPosition(-200);
        setAllWheelMotorPower(0.5);
        setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
        setAllWheelMotorPower(0);

        // The parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
            // Strafe left
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(2000);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
            // Strafe left slightly
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeLeft(500);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        } else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
            // Strafe right
            setAllWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            strafeRight(750);
            setAllWheelMotorPower(1);
            setAllWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            trackAllWheelCurrentPositionTelemetryWhileMotorIsBusy();
            setAllWheelMotorPower(0);
        }
    }
}
