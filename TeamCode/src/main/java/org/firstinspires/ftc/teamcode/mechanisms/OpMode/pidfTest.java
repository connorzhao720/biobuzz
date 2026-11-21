package org.firstinspires.ftc.teamcode.mechanisms.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.hardware.RobotHardware;


@TeleOp(name = "pidf test")
public class pidfTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    private final double SHOOTER_RPM_SHORT = 1455.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 1840; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;

    private double controller1Speed = 1;

    double rpm = 1050.0;
    private double kp = 40;
    private double ki = 0.355;
    private double kd = 6.6;

    int selectedState = 0;
    double pidStep = 0.0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;
            boolean x = gamepad1.x;


            if (gamepad1.left_bumper) {
                robot.spinBothIntakes();
            }

            if (gamepad1.left_trigger >= 0.7) {
                robot.spinFirstOnly();
            }

            if (gamepad1.right_trigger >= 0.7) {
                robot.spinOutBothIntakes();
            }

            if (gamepad1.right_bumper) {
                robot.stopIntake();
            }
            //set 0
            if (gamepad1.y) {
                robot.shoot_y();
            }

            if (gamepad1.b) {
                robot.shoot_b();
            }

            if (gamepad1.x) {
                robot.stopShooter();
            }


            if (gamepad2.dpad_up && selectedState >= 2) {
                selectedState = 0;
                sleep(500);
            } else if (gamepad2.dpad_up) {
                selectedState++;
                sleep(500);
            } else if (gamepad2.dpad_down && selectedState <= 0) {
                selectedState = 2;
                sleep(500);
            } else if (gamepad2.dpad_down) {
                selectedState--;
                sleep(500);
            }

            if (gamepad2.dpad_left) {
                if (selectedState == 0) {
                    kp -= pidStep;
                    sleep(500);
                } else if (selectedState == 1) {
                    ki -= pidStep;
                    sleep(500);
                } else if (selectedState == 2) {
                    kd -= pidStep;
                    sleep(500);
                }
            } else if (gamepad2.dpad_right) {
                if (selectedState == 0) {
                    kp += pidStep;
                    sleep(500);
                } else if (selectedState == 1) {
                    ki += pidStep;
                    sleep(500);
                } else if (selectedState == 2) {
                    kd += pidStep;
                    sleep(500);
                }
            }


            if (gamepad2.x) pidStep = 0.01;
            else if (gamepad2.a) pidStep = 0.1;
            else if (gamepad2.b) pidStep = 1.0;
            else if (gamepad2.y) pidStep = 10.0;

            robot.motorshoot.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kp, ki, kd, 11.5)
            );

            telemetry.addData("kp", kp);
            telemetry.addData("ki", ki);
            telemetry.addData("kd", kd);
            telemetry.addData("pidStep: ", pidStep);
            telemetry.addData("selectedState: ", selectedState);
            telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
            telemetry.addData("shooter ideal", rpm);
            telemetry.update();

            double x_dir = -gamepad1.left_stick_y * 1.3 * controller1Speed;
            double y_dir = gamepad1.left_stick_x * 1.3 * controller1Speed;
            double turn = gamepad1.right_stick_x * 1.3 * controller1Speed;

            double flPower = x_dir + y_dir + turn;
            double blPower = y_dir - x_dir - turn;//+turn
            double frPower = y_dir - x_dir + turn;// -turn
            double brPower = y_dir + x_dir - turn;


            if (gamepad1.dpad_down) {
                controller1Speed = 0.3;
                robot.setDrivePower(-flPower, -frPower, -blPower, -brPower);
            } else {
                controller1Speed = 1;
                robot.setDrivePower(-flPower, -frPower, -blPower, -brPower);
            }

            idle();

            // --- TELEMETRY ---
//            telemetry.addData("intakePower", robot.motorintake != null ? robot.motorintake.getPower() : "null");


        }


    }

}
