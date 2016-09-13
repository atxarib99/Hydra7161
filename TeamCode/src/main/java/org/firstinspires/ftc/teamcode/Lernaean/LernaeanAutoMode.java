package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Lernaean.Interfaces.DriveInterface;
import org.firstinspires.ftc.teamcode.Lernaean.Interfaces.ServoInterface;

/**
 * Created by Arib on 9/13/2016.
 */
public abstract class LernaeanAutoMode extends LinearOpMode implements DriveInterface, ServoInterface {

    BNO055IMU gyro;
    Orientation angles;
    Acceleration accel;

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    Servo rightRight;
    Servo rightLeft;
    Servo leftRight;
    Servo leftLeft;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsMiddle;
    OpticalDistanceSensor odsSide;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    int nullValue = 0;
    double angleError;

    private final double RIGHT_RIGHT_OUT = 1;
    private final double RIGHT_RIGHT_IN = 0;
    private final double RIGHT_LEFT_OUT = 1;
    private final double RIGHT_LEFT_IN = 0;
    private final double LEFT_RIGHT_OUT = 0;
    private final double LEFT_RIGHT_IN = 1;
    private final double LEFT_LEFT_OUT = 0;
    private final double LEFT_LEFT_IN = 1;


    public void map() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBR = hardwareMap.dcMotor.get("BR");
        rightRight = hardwareMap.servo.get("RRS");
        rightLeft = hardwareMap.servo.get("RLS");
        leftRight = hardwareMap.servo.get("LRS");
        leftLeft = hardwareMap.servo.get("LLS");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rightColor = hardwareMap.colorSensor.get("rCol");
        leftColor = hardwareMap.colorSensor.get("lCol");
        odsMiddle = hardwareMap.opticalDistanceSensor.get("odsM");
        odsSide = hardwareMap.opticalDistanceSensor.get("odsS");
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

    public void rightRightOut() {
        rightRight.setPosition(RIGHT_RIGHT_OUT);
    }

    public void rightRightIn() {
        rightRight.setPosition(RIGHT_RIGHT_IN);
    }

    public void rightLeftOut() {
        rightLeft.setPosition(RIGHT_LEFT_OUT);
    }

    public void rightLeftIn() {
        rightLeft.setPosition(RIGHT_LEFT_IN);
    }

    public void leftLeftOut() {
        leftLeft.setPosition(LEFT_LEFT_OUT);
    }

    public void leftLeftIn() {
        leftLeft.setPosition(LEFT_LEFT_IN);
    }

    public void leftRightOut() {
        leftRight.setPosition(LEFT_RIGHT_OUT);
    }

    public void leftRightIn() {
        leftRight.setPosition(LEFT_RIGHT_IN);
    }

    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        //notifies Driver Station of current event
        telemetry.update();

        //resets the Gyro's angle
        resetGyro();
        double angle;

        setNullValue();

        int currentEncoder = getEncoderAvg() - nullValue;
        //while target is not reached
        while((encoderVal > currentEncoder)) {
            getAngles();
            angle = getGyroYaw();

            currentEncoder = getEncoderAvg() - nullValue;

            //if off to the left, correct
            if(angle > 2) {
                startMotors(pow, (pow * .75));
                telemetry.update();
            } else if(angle < -2) { //if off to the right, correct
                startMotors((pow * .75), pow);
                telemetry.update();
            } else { //if heading is fine keep moving straight
                startMotors(pow, pow);
                telemetry.update();
            }

        }

        //once finished stop moving and send data
        stopMotors();
        getAngles();
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
        }
        telemetry.update()
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
        }
        telemetry.update()
        stopMotors();                                  //stop motion
//        Double d = angle;
//        int rotated = d.intValue();
//        Double ticks = rotated / 90.0;
//        int ticksI = ticks.intValue();
//        updateFacing(ticksI);
    }

    public boolean middleLine() {
        return odsMiddle.getLightDetected() > .5;
    }

    public boolean sideLine() {
        return odsSide.getLightDetected() > .5;
    }

    //=============BEGIN CALCULATION METHODS==========================

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





}
