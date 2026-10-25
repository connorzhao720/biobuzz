package org.firstinspires.ftc.teamcode.mechanisms.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.hardware.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.util.function.Supplier;


@TeleOp(name = "OpMode3")
public class OpMode3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    private final double SHOOTER_RPM_SHORT = 1455.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 1840; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;

    private double controller1Speed = 1;



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

            double x_dir = -gamepad1.left_stick_y * 1.3 * controller1Speed;
            double y_dir = gamepad1.left_stick_x * 1.3 * controller1Speed;
            double turn = -gamepad1.right_stick_x * 1.3 * controller1Speed;

            double flPower = x_dir + y_dir + turn;
            double blPower = y_dir - x_dir + turn;
            double frPower = y_dir - x_dir - turn;
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
            telemetry.addData("shooter velocity", robot.motorshoot != null ? robot.motorshoot.getVelocity() : "null");
            telemetry.addData("shooter tpr", robot.motorshoot != null && robot.motorshoot.getMotorType() != null ? robot.motorshoot.getMotorType().getTicksPerRev() : "null");
            telemetry.update();

        }


    }

}
