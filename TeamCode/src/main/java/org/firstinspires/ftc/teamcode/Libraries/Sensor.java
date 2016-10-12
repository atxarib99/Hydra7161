package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Arib on 10/10/2016.
 */
public class Sensor {

    BNO055IMU gyro;
    OpticalDistanceSensor mid;
    OpticalDistanceSensor side;
    ColorSensor colorR;
    ColorSensor colorL;
    LinearOpMode opMode;
    Orientation angles;
    BNO055IMU.Parameters parameters;
    public Sensor(LinearOpMode opMode) {
        this.opMode = opMode;
        mid = opMode.hardwareMap.opticalDistanceSensor.get("mid");
        side = opMode.hardwareMap.opticalDistanceSensor.get("side");
        colorR = opMode.hardwareMap.colorSensor.get("colorR");
        colorL = opMode.hardwareMap.colorSensor.get("colorL");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

    }

    public double getGyroYaw() {
        return angles.firstAngle;
    }

    public boolean isRightRed() {
        return colorR.red() > 450;
    }

    public boolean isLeftRed() {
        return colorL.red() > 450;
    }

    public boolean isMidLine() {
        return mid.getRawLightDetected() > 2;
    }

    public boolean isSideLine() {
        return side.getRawLightDetected() > 2;
    }

    public boolean resetGyro() {
        return gyro.initialize(parameters);
    }
}
