package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Arib on 10/10/2016.
 */
public class Sensor {

    BNO055IMU gyro;
    OpticalDistanceSensor mid;
    OpticalDistanceSensor side;
    ColorSensor color;
    LinearOpMode opMode;
    public Sensor(LinearOpMode opMode) {
        this.opMode = opMode;
        mid = opMode.hardwareMap.opticalDistanceSensor.get("mid");
        side = opMode.hardwareMap.opticalDistanceSensor.get("side");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    }
}
