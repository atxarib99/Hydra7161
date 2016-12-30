package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Arib on 9/22/2016.
 */
public abstract class TrollOpMode extends OpMode {
    DcMotor motorBL;
    DcMotor motorFR;
//    BNO055IMU gyro;
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    Orientation angles;
//    Acceleration accel;

    //ColorSensor leftColor;


    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFR = hardwareMap.dcMotor.get("FR");
        //leftColor = hardwareMap.colorSensor.get("lcolor");
//        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationData     = gyro.readCalibrationData();
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        gyro.initialize(parameters);
//        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        accel = gyro.getGravity();
        hardwareMap.logDevices();
        telemetry.addData("init", "finished");
    }

    public abstract void loop();

    @Override
    public void stop() {
        stopMotors();
    }

    public void startMotors(double ri, double le) {
        motorBL.setPower(le);
        motorFR.setPower(-ri);
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorFR.setPower(0);
    }

}
