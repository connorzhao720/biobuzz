package org.firstinspires.ftc.teamcode.mechanisms.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.hardware.RobotHardware;


@TeleOp(name = "OpMode")
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

    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double kf = 11.5;
    int current = 0;
    int change = 1;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;
            boolean x = gamepad1.x;



            if (gamepad1.left_bumper){
                robot.spinBothIntakes();
            }

            if (gamepad1.left_trigger >= 0.7){
                robot.spinSecondOnly();
            }

            if(gamepad1.right_trigger >= 0.7){
                robot.spinOutBothIntakes();
            }

            if(gamepad1.right_bumper){
                robot.stopIntake();
            }
            //set 0
            if(gamepad1.y){
                robot.shoot_y();
            }

            if(gamepad1.b){
                robot.shoot_b();
            }

            if(gamepad1.x){
                robot.shoot_x();
            }

            if(gamepad2.a){
                if(current == 0){
                    kp = kp + (1 * change);
                }
                if(current == 1){
                    ki = ki + (1 * change);
                }
                if(current == 2){
                    kd = kd + (1 * change);
                }
            }

            if(gamepad2.b){
                if(current == 0){
                    kp = kp + (10 * change);
                }
                if(current == 1){
                    ki = ki + (10 * change);
                }
                if(current == 2){
                    kd = kd + (10 * change);
                }
            }

            if(gamepad2.x){
                if(current == 0){
                    kp = kp + (0.1 * change);
                }
                if(current == 1){
                    ki = ki + (0.1 * change);
                }
                if(current == 2){
                    kd = kd + (0.1 * change);
                }
            }

            if(gamepad2.y){
                change = -change;


            if(gamepad2.dpad_up){
                current = 0;
            }
            if(gamepad2.dpad_right){
                current = 1;
            }

            if(gamepad2.dpad_down){
                current = 2;
            }

            telemetry.addData("shooter velocity", robot.motorshoot != null ? robot.motorshoot.getVelocity() : "null");
            telemetry.addData("shooter tpr", robot.motorshoot != null && robot.motorshoot.getMotorType() != null ? robot.motorshoot.getMotorType().getTicksPerRev() : "null");
            telemetry.addData("kp", kp);
            telemetry.addData("ki", ki);
            telemetry.addData("kd", kd);
            telemetry.addData("kf", kf);
            telemetry.addData("change", change);
            telemetry.addData("current", current);
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
            }
            else {
                controller1Speed = 1;
                robot.setDrivePower(-flPower, -frPower, -blPower, -brPower);
            }

            idle();

            // --- TELEMETRY ---
//            telemetry.addData("intakePower", robot.motorintake != null ? robot.motorintake.getPower() : "null");


        }


    }

}}
