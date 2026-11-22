
package org.firstinspires.ftc.teamcode.mechanisms.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.hardware.RobotHardware;

@Autonomous(name = "autonomous")
public class autonomous extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    // Encoder calibration
    double ticksPerInch = 31.3;
//    public DcMotorEx shooter = robot.motorshoot;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.setAutoDriveMotorMode();

        telemetry.addLine("Autonomous ready");
        telemetry.update();

        waitForStart();

        int oneTile = (int) Math.ceil(ticksPerInch * 23.0);

        if (opModeIsActive()) {

            // STEP 1: Drive forward 3 tiles
            driveMotors(oneTile * 3, oneTile * 3, oneTile * 3, oneTile * 3, 0.35, true, robot.getCurrentYaw());

            // STEP 2: Turn 90 degrees
            double currentYaw = robot.imu
                    .getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            double targetYaw = currentYaw + 90;

            // Normalize target angle
            while (targetYaw > 180) targetYaw -= 360;
            while (targetYaw < -180) targetYaw += 360;

            turnToTargetYaw(targetYaw, 0.35, 3000);

            // STEP 3: Drive backward 1 tile
            driveMotors(-oneTile, -oneTile, -oneTile, -oneTile, 0.35, true, robot.getCurrentYaw());

//           shoot();
//            double targetTicksPerSecond = ;
//            double tolerance = ;
//            shooter.setVelocity(targetTicksPerSecond);
//            long startTime = System.currentTimeMillis();
//            while (opModeIsActive() &&
//                    Math.abs(robot.motorshoot.getVelocity()
//                            - targetTicksPerSecond) > tolerance && System.currentTimeMillis() - startTime < 3000) {
//
//                telemetry.addData("Shooter velocity",
//                        robot.motorshoot.getVelocity());
//                telemetry.update();
//                idle();
//            }
        }

        stopDriveMotors();
    }

    private void driveMotors(
            int flTarget,
            int blTarget,
            int frTarget,
            int brTarget,
            double power,
            boolean bKeepYaw,
            double targetYaw) {

        double currentYaw;
        double yawError;
        double powerDeltaPct;
        double powerL;
        double powerR;

        int direction;

        // Reset encoders
        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        // Set run mode
        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // RUN_TO_POSITION uses positive power magnitudes.
        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        // Heading correction only works for straight movements
        if (!(flTarget == blTarget &&
                flTarget == frTarget &&
                flTarget == brTarget)) {

            bKeepYaw = false;
        }

        direction = (flTarget > 0) ? 1 : -1;

        // Movement timeout
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < timeout &&
                (robot.motorfl.isBusy() ||
                        robot.motorbl.isBusy() ||
                        robot.motorfr.isBusy() ||
                        robot.motorbr.isBusy())) {

            if (bKeepYaw) {

                currentYaw = robot.imu
                        .getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES);

                // Normalize heading error
                yawError = targetYaw - currentYaw;

                while (yawError > 180) yawError -= 360;
                while (yawError < -180) yawError += 360;

                // Calculate correction amount
                powerDeltaPct = Math.min(
                        Math.abs(yawError) / 2.0 * 0.25,
                        0.25
                );

                if (yawError > 0) {

                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);

                } else {

                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }

                // Keep power within valid limits
                powerL = Math.max(0, Math.min(1.0, powerL));
                powerR = Math.max(0, Math.min(1.0, powerR));

                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }

            idle();
        }

        stopDriveMotors();
    }


    private void driveStrafe(
            int flTarget,
            int blTarget,
            int frTarget,
            int brTarget,
            double power,
            boolean bKeepYaw,
            double targetYaw) {

        // Reset encoders
        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        // Set run mode
        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Positive power magnitudes.
        // Encoder target signs determine each wheel's direction.
        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        // Timeout
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        // Heading correction is intentionally not applied here.
        // Strafing requires a different correction calculation.

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < timeout &&
                (robot.motorfl.isBusy() ||
                        robot.motorbl.isBusy() ||
                        robot.motorfr.isBusy() ||
                        robot.motorbr.isBusy())) {

            idle();
        }

        stopDriveMotors();
    }


    private void turnToTargetYaw(
            double targetYawDegree,
            double power,
            long maxAllowedTimeInMills) {

        long startTime = System.currentTimeMillis();

        double currentYaw;
        double error;

        // Switch out of RUN_TO_POSITION before turning
        robot.motorfl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime <
                        maxAllowedTimeInMills) {

            currentYaw = robot.imu
                    .getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            // Calculate and normalize angle error
            error = targetYawDegree - currentYaw;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Stop when close enough
            if (Math.abs(error) < 1.5) {
                break;
            }

            // Proportional turning power
            double turnPower = Math.abs(error) * 0.012;

            turnPower = Math.min(turnPower, power);
            turnPower = Math.max(turnPower, 0.12);

            // Determine turn direction
            double direction = Math.signum(error);

            // Opposite motor directions rotate the robot
            robot.motorfl.setPower(-direction * turnPower);
            robot.motorbl.setPower(-direction * turnPower);
            robot.motorfr.setPower(direction * turnPower);
            robot.motorbr.setPower(direction * turnPower);

            telemetry.addData("Current yaw", currentYaw);
            telemetry.addData("Target yaw", targetYawDegree);
            telemetry.addData("Error", error);
            telemetry.addData("Turn power", turnPower);
            telemetry.update();

            idle();
        }

        stopDriveMotors();
    }


    private void stopDriveMotors() {

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }

    public void shoot(double kp, double ki, double kd){
        robot.motorshoot.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, 11.5)
        );
    }
}
