package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;
import android.widget.AdapterViewFlipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 6/29/2016.
 */
public class AutonomousPractice extends LinearOpMode {

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor motorBL;

    Servo climbers;
    Servo paddles;

    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        climbers = hardwareMap.servo.get("climbers");
        paddles = hardwareMap.servo.get("paddles");

        climbers.setPosition(0);
        paddles.setPosition(1);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while(time.time() < 5.0) {
            motorBL.setPower(-1);
            motorBR.setPower(1);
            motorFR.setPower(1);
            motorFL.setPower(-1);
        }
        motorBL.getCurrentPosition();
        int targetEncoder = 1440;
        while(motorBL.getCurrentPosition() < targetEncoder) {
            motorBL.setPower(-1);
            motorBR.setPower(-1);
            motorFR.setPower(-1);
            motorFL.setPower(-1);
        }
        climbers.setPosition(1);
        AdafruitIMU gyro;
        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage() + "THIS IS THE ERROR");
        }
        gyro.startIMU();
        double targetAngle = 90.0;
        double currentAngle = yawAngle[0];
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        while(currentAngle < targetAngle) {
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            motorBL.setPower(-1);
            motorBR.setPower(-1);
            motorFR.setPower(-1);
            motorFL.setPower(-1);
            currentAngle = yawAngle[0];
        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);

    }
}
