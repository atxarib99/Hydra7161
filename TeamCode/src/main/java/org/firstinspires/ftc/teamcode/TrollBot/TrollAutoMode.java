package org.firstinspires.ftc.teamcode.TrollBot;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Arib on 9/22/2016.
 */
public abstract class TrollAutoMode extends LinearOpMode {
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

    ColorSensor rightColor;
    ColorSensor leftColor;

    int nullValue;

    double angleError;


    public void map() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        mid = hardwareMap.opticalDistanceSensor.get("mid");
        side = hardwareMap.opticalDistanceSensor.get("side");
        rightColor = hardwareMap.colorSensor.get("rcolor");
        leftColor = hardwareMap.colorSensor.get("lcolor");
        composeTelemetry();
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        nullValue = 0;
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        accel = gyro.getGravity();

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

    public void moveForwardPID(double pow, int encoderVal) throws InterruptedException {

        resetGyro();
        double angle;
        telemetry.update();

        double error;
        double power;
        setNullValue();

        int currentEncoder = getEncoderAvg() - nullValue;
        while (encoderVal > currentEncoder) {
            telemetry.update();
            getAngles();
            angle = getGyroYaw();

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) + .25;

            Range.clip(power, -1, 1);

            telemetry.addData("Power", power);
            telemetry.addData("LeftPower", motorBL.getPower());
            telemetry.addData("RightPower", motorBR.getPower());

            if (angle > 2) {
                startMotors((power * .75), power);
                telemetry.update();
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");
            } else if (angle < -2) { //if off to the right, correct
                startMotors(power, (power * .75));
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");
                telemetry.update();
            } else { //if heading is fine keep moving straight
                startMotors(power, power);
                telemetry.update();
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");

            }
            idle();
        }
        stopMotors();
        telemetry.update();
        angleError = getGyroYaw();
    }

    //P loop for PID //TODO: UPDATE VALS AND TEST
    public void pRotate(double pow, double angle) throws InterruptedException {
        resetGyro();
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        double error;
        getAngles();
        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;
        //do while
        while (Math.abs(currentAngle) < angleTo - 2) {
            getAngles();                                //update angles
            currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if(power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }
            startMotors(-power, power);                 //set the motors to turn
            telemetry.addData("PID", power);
            telemetry.update();
            previousError = error;
            idle();
        }
        telemetry.update();
        stopMotors();                                  //stop motion
//        Double d = angle;
//        int rotated = d.intValue();
//        Double ticks = rotated / 90.0;
//        int ticksI = ticks.intValue();
//        updateFacing(ticksI);
    }

    public void pRotateNoReset(double pow, double angle) throws InterruptedException {
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        double error;
        getAngles();
        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;
        //do while
        while (Math.abs(currentAngle) < angleTo - 2) {
            getAngles();                                //update angles
            currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if(power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }
            startMotors(-power, power);                 //set the motors to turn
            telemetry.addData("PID", power);
            telemetry.update();
            previousError = error;
            idle();
        }
        telemetry.update();
        stopMotors();                                  //stop motion
//        Double d = angle;
//        int rotated = d.intValue();
//        Double ticks = rotated / 90.0;
//        int ticksI = ticks.intValue();
//        updateFacing(ticksI);
    }


    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))
                + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition())))
                / 4;
    }
    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public void getAngles() {
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    public boolean isRightRed() {
        if(rightColor.red() > 450) {
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
                        return (getEncoderAvg() - nullValue) + "";
                    }
                });
    }

    public boolean middleLine() {
        return mid.getLightDetected() > 2;
    }

    public boolean sideLine() {
        return side.getLightDetected() > 2;
    }

    public void moveTillMid() {
        while (!middleLine() && !sideLine()){
            startMotors(.5, .5);
        }
        stopMotors();
    }

    public void straightenRedSide() {
        while (!sideLine()){
            startMotors(-.5, .5);
        }
    }

    public void straightenBlueSide() {
        while (!sideLine()){
            startMotors(.5, -.5);
        }
    }
}
