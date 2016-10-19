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
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;
    LinearOpMode opMode;
    Orientation angles;
    BNO055IMU.Parameters parameters;
    public Sensor(LinearOpMode opMode) {
        this.opMode = opMode;
        right = opMode.hardwareMap.opticalDistanceSensor.get("odsR");
        left = opMode.hardwareMap.opticalDistanceSensor.get("odsL");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //gyro = this.opMode.hardwareMap.get(BNO055IMU.class, "gyro");
        resetGyro();
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    public double getGyroYaw() {
        return angles.firstAngle;
    }

    public boolean isRightLine() {
        return right.getRawLightDetected() > 2;
    }

    public boolean isLeftLine() {
        return left.getRawLightDetected() > 2;
    }

    public boolean resetGyro() {
        return gyro.initialize(parameters);
    }
}
