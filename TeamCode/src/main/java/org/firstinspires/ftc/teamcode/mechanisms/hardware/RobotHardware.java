package org.firstinspires.ftc.teamcode.mechanisms.hardware;


import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
*/

public class RobotHardware {
    HardwareMap hwMap =  null;

    public IMU imu;
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;
    public DcMotor motorbl = null;
    public DcMotor motorintake = null;
    //public Limelight3A limelight;

    // Use DcMotorEx for shooter so we can control velocity (ticks/sec)
    public DcMotorEx motorshoot = null;
    public DcMotorEx motorturret = null;
    public DcMotor motorsecondintake = null;
    public Servo servoblocker = null;
    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;
    public double rpm = 580;//x1.5 bc ratio
    public double servoOpen = 0.15;
    public double servoClose = 0.7;

    public RobotHardware() {}
    public void init(HardwareMap ahwMap)    {
        hwMap = ahwMap;

        motorfl = hwMap.get(DcMotor.class, "fl");
        motorfr = hwMap.get(DcMotor.class, "fr");
        motorbl = hwMap.get(DcMotor.class, "bl");
        motorbr = hwMap.get(DcMotor.class, "br");

        motorintake = hwMap.get(DcMotor.class, "firstintake");
        motorsecondintake = hwMap.get(DcMotor.class, "secondintake");

        servoblocker = hwMap.get(Servo.class, "blocker");
//        motorsweep = hwMap.get(DcMotor.class, "sweep");

        // get shooter as DcMotorEx to expose velocity control
        motorshoot = hwMap.get(DcMotorEx.class, "shooter");
        double kp=104.92;
        double ki = 0.18;
        double kd = 2.29;
        double kf = 11.5;

        motorshoot.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp,ki,kd,kf)
        );

        //limelight = hwMap.get(Limelight3A.class, "limelight");

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorbr.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        motorintake.setDirection(DcMotor.Direction.REVERSE);
        motorsecondintake.setDirection(DcMotor.Direction.REVERSE);

//        motorintake.setDirection(DcMotor.Direction.REVERSE);
//        motorsweep.setDirection(DcMotor.Direction.REVERSE);




        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorshoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorshoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorshoot.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    public void setAutoDriveMotorMode() {
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }

    public double increaserpm(){
        return (rpm + 100);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }


    public void spinBothIntakes(){
        motorintake.setPower(0.7);
        motorsecondintake.setPower(-0.7);
    }
    public void spinFirstOnly(){
        motorsecondintake.setPower(0.0);
        motorintake.setPower(0.7);
    }
    public void spinOutBothIntakes(){
        motorintake.setPower(-0.7);
        motorsecondintake.setPower(0.7);
    }
    public void setAllDrivePower(double p){ setDrivePower(p,p,p,p);}

    // Shooter helpers using DcMotorEx velocity control (ticks/sec)
    public void startShooterAtRPM(double rpm) {
        if (motorshoot == null) return;
       // convert RPM to ticks/sec
        double ticksPerRev = 56; //FIX  2786
        double ticksPerSec = ticksPerRev * rpm / 60.0;
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorshoot.setVelocity(ticksPerSec);
    }

    public double getShooterRPM() {
        if (motorshoot == null) return 0.0;
        double ticksPerSec = motorshoot.getVelocity();
        double ticksPerRev = motorshoot.getMotorType().getTicksPerRev();
        return ticksPerSec * 60.0 / ticksPerRev;
    }
    public void stopIntake(){
        motorintake.setPower(0);
        motorsecondintake.setPower(0);
    }
    public void stopShooter() {
        if (motorshoot == null) return;
       motorshoot.setPower(0.0);
       motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       servoblocker.setPosition(servoClose);
    }
    public void shoot_b(){
        motorshoot.setVelocity(rpm);
    }
    public void shoot_y(){
        servoblocker.setPosition(servoOpen);
    }

    public void autoShoot(){
        motorshoot.setVelocity(rpm);
        while(motorshoot.getVelocity() > (rpm - 50)){
            servoblocker.setPosition(servoOpen);
            spinBothIntakes();
        }
    }


//    public void autoShootShort(){
//        if(motorshoot.getVelocity() > 1430){
////            motorintake.setPower(-1.0);
////            motorsweep.setPower(0.8);
//            sleep(400);
////            motorintake.setPower(0.0);
////            motorsweep.setPower(0.0);
//        }
//    }
//
//    public void autoShootLong(){
//        if(motorshoot.getVelocity() > 1785){
////            motorintake.setPower(-1.0);
////            motorsweep.setPower(0.8);
//            sleep(400);
////            motorintake.setPower(0.0);
////            motorsweep.setPower(0.0);
//        }
//    }
/*

    private void wheelIndividualTest() {
        boolean b = gamepad1.b;
        boolean ddown = gamepad1.dpad_down;

        if (b && ddown) {
            robot.motorfl.setPower(1);
            sleep(500);

            robot.motorfl.setPower(0);
            robot.motorfr.setPower(1);

            sleep(500);
            robot.motorfr.setPower(0);
            robot.motorbr.setPower(1);

            sleep(500);
            robot.motorbr.setPower(0);
            robot.motorbl.setPower(1);

            sleep(500);
            robot.motorbl.setPower(0);
        }


    }

 */
}
