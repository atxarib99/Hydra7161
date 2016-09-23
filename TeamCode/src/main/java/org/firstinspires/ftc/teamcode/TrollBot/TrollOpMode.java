package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
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
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DeviceInterfaceModule dim;
    OpticalDistanceSensor mid;
    OpticalDistanceSensor side;
    BNO055IMU gyro;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;
    Acceleration accel;

    //ColorSensor rightColor;
    //ColorSensor leftColor;

    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        mid = hardwareMap.opticalDistanceSensor.get("mid");
        side = hardwareMap.opticalDistanceSensor.get("side");
        //rightColor = hardwareMap.colorSensor.get("rcolor");
        //leftColor = hardwareMap.colorSensor.get("lcolor");
        composeTelemetry();
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        accel = gyro.getGravity();
    }

    public abstract void loop();

    @Override
    public void stop() {
        stopMotors();
    }

    public void startMotors(double ri, double le) {
        motorBL.setPower(-le);
        motorBR.setPower(ri);
        motorFL.setPower(-le);
        motorFR.setPower(ri);
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))
                + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition())))
                / 4;
    }

    public void getAngles() {
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    /*    if(rightColor.red() > 450) {
            return true;
        }
        return false;
    }


    public boolean isLeftRed() {
        if(leftColor.red() > 450) {
            return true;
        }
        return false;
    }
    */
    public double getGyroYaw() {
        return (double) angles.firstAngle;
    }

    public void resetGyro() {
        gyro.initialize(parameters);
    }

    public void composeTelemetry() {

        telemetry.addLine()
                .addData("yaw", new Func<String>() {
                    @Override public String value() {
                        return angles.firstAngle + "";
                    }
                });

        telemetry.addLine()
                .addData("encoder", new Func<String>() {
                    @Override public String value() {
                        return (getEncoderAvg()) + "";
                    }
                });

        /* telemetry.addLine()
                .addData("light", new Func<String>)(){
                    @Override public String value() {
                        return
            }
        */
        }
    }
}
