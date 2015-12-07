package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 11/30/2015.
 */
public class QualifierAutonomous extends LinearOpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    int BLencoder;
    int BRencoder;
    int BRnullEncoder;
    int BLnullEncoder;
    AdafruitIMU gyro;
    Servo rightBar;
    Servo leftBar;
    boolean frontWheels;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    public void startMotors(double power1, double power2, double power3, double power4) {
        motorBR.setPower(power1);
        motorBL.setPower(power2);
        motorFL.setPower(power3);
        motorFR.setPower(power4);
    }
    public int getEncoderAvg() {
        return (Math.abs(motorBL.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorFR.getCurrentPosition())) / 4;
    }
    public void stopMotors() {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void resetEncoders() {
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    public void getEncoderValues() {
        telemetry.addData("motorBL", motorBL.getCurrentPosition());

        telemetry.addData("motorFR", motorFR.getCurrentPosition());

        telemetry.addData("motorBR", motorBR.getCurrentPosition());

        telemetry.addData("motorFL", motorFL.getCurrentPosition());
    }
    public int getBackWheelAvg() {
        return (Math.abs(BLencoder - BLnullEncoder) + Math.abs(BRencoder - BRnullEncoder)) / 2;
    }
    @Override
    public void runOpMode() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        rightBar = hardwareMap.servo.get("rightBar");
        leftBar = hardwareMap.servo.get("leftBar");
        frontWheels = false;
        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"
                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMUAccel.BNO055_ADDRESS_A * 2) //By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMUAccel.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {

        }
        resetEncoders();
        gyro.startIMU();
        int currentEncoder = 0;
        BLnullEncoder = 0;
        BRnullEncoder = 0;
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double currentAngle = yawAngle[0];
        while(currentEncoder < 8500) {
            startMotors(1, -1, 0, 0);
            currentEncoder = getBackWheelAvg();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            currentAngle = yawAngle[0];
            if(currentAngle > 2) {
                resetEncoders();
                while(currentAngle > 2) {
                    startMotors(.2, .2, 0, 0);
                    BLnullEncoder = Math.abs(motorBL.getCurrentPosition());
                    BRnullEncoder = Math.abs(motorBR.getCurrentPosition());
                }
            }
            if (currentAngle < -2) {
                resetEncoders();
                while(currentAngle < -2) {
                    startMotors(-.2, -.2, 0, 0);
                    BLnullEncoder = Math.abs(motorBL.getCurrentPosition());
                    BRnullEncoder = Math.abs(motorBR.getCurrentPosition());
                }
            }
        }
        stopMotors();
        resetEncoders();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        currentAngle = yawAngle[0];
        while (currentAngle < 90) {
            startMotors(-1, -1, 0, 0);
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            currentAngle = yawAngle[0];
        }
        stopMotors();
        resetEncoders();
        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while(time.time() < 2.5) {
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            if(pitchAngle[0] > 25 && !frontWheels) {
                frontWheels = true;
            }
            if(frontWheels) {
                startMotors(1, -1, -1, 1);
            }
            else
                startMotors(1, -1, 0, 0);
        }
        stopMotors();

    }
}
