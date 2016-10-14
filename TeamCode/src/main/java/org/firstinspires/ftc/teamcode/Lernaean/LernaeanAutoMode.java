package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
/**
 * Created by Arib on 9/13/2016.
 */
public abstract class LernaeanAutoMode extends LinearOpMode {

    BNO055IMU gyro;
    Orientation angles;
    Acceleration accel;

    DcMotor motorL;
    DcMotor motorR;
    Servo back;
    Servo front;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsMiddle;
    OpticalDistanceSensor odsSide;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    int nullValue = 0;
    double angleError;

    private final double FRONT_OUT = 1;
    private final double FRONT_SEMI = .5;
    private final double FRONT_IN = 0;
    private final double BACK_OUT = 1;
    private final double BACK_SEMI = .5;
    private final double BACK_IN = 0;
    private int xTile;
    private int yTile;
    int facing;
    private final double WHEEL_DIAMETER = 4;
    private final double DISTANCE_PER_ROTATION = WHEEL_DIAMETER * Math.PI;
    private final int SINGLE_ROTATION = 1120;

    public void map() {
        map(6, 3, 1); //TODO: UPDATE THESE PLACEHOLDERS
    }


    public void map(int x, int y, int f) {
        xTile = x;
        yTile = y;
        facing = f;
        motorL = hardwareMap.dcMotor.get("L");
        motorR = hardwareMap.dcMotor.get("R");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
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

    //===============BEGIN MOVEMENT METHODS=============

    public void startMotors(double ri, double le) {
        motorL.setPower(-le);
        motorR.setPower(ri);
    }

    public void stopMotors() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    public void frontOut() {
        front.setPosition(FRONT_OUT);
    }

    public void frontSemi() {
        front.setPosition(FRONT_SEMI);
    }

    public void frontIn() {
        front.setPosition(FRONT_IN);
    }

    public void backOut() {
        back.setPosition(BACK_OUT);
    }

    public void backSemi() {
        back.setPosition(BACK_SEMI);
    }

    public void backIn() {
        back.setPosition(BACK_IN);
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
            idle();
        }

        //once finished stop moving and send data
        stopMotors();
        getAngles();
        angleError = getGyroYaw();
    }

    public void moveForwardPID(double pow, int encoderVal) throws InterruptedException {

        resetGyro();
        double angle;
        telemetry.update();

        double error;
        double power;
        setNullValue();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder) {
            telemetry.update();
            getAngles();
            angle = getGyroYaw();

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) + .25;

            Range.clip(power, -1, 1);

            telemetry.addData("Power", power);
            telemetry.addData("LeftPower", motorL.getPower());
            telemetry.addData("RightPower", motorR.getPower());

            if(angle > 2) {
                startMotors((power * .75), power);
                telemetry.update();
                telemetry.addData("LeftPower", motorL.getPower() + "");
                telemetry.addData("RightPower", motorR.getPower() + "");
            } else if(angle < -2) { //if off to the right, correct
                startMotors(power, (power * .75) );
                telemetry.addData("LeftPower", motorL.getPower() + "");
                telemetry.addData("RightPower", motorR.getPower() + "");
                telemetry.update();
            } else { //if heading is fine keep moving straight
                startMotors(power, power);
                telemetry.update();
                telemetry.addData("LeftPower", motorL.getPower() + "");
                telemetry.addData("RightPower", motorR.getPower() + "");

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

    public boolean middleLine() {
        return odsMiddle.getLightDetected() > 2;
    }

    public boolean sideLine() {
        return odsSide.getLightDetected() > 2;
    }

    //=============END MOVEMENT METHODS===========================
    //=============BEGIN GRID METHODS=================
    public void setFacing(int f) throws InterruptedException {
        int diff = f - facing;
        double angle = 90 * (Math.abs(diff));
        double pow = .2;
        if(angle > 0) {
            pow = .2;
        }
        else if(angle < 0) {
            pow = -.2;
        }

        if(diff != 0)
            pRotate(pow, angle);
    }

    public void moveXTiles(int numTiles) throws InterruptedException {
        double oneTileInInches = 24;
        Double distToMoveInches = oneTileInInches * numTiles;
        double rotationsToMove = distToMoveInches / DISTANCE_PER_ROTATION;
        int encoderTicksToMove = (int)Math.round(rotationsToMove * SINGLE_ROTATION);
        moveForwardPID(.5, encoderTicksToMove);
        stopMotors();
    }

    public void moveToCoordinatePos(int xTileTo, int yTileTo) throws InterruptedException {
        int yDiff = yTileTo - yTile;
        int xDiff = xTileTo - xTile;
        if(yDiff > 0) {
            setFacing(1);
        } else if(yDiff < 0) {
            setFacing(3);
        }

        moveXTiles(Math.abs(yDiff));
        pRotateNoReset(-.2, 0);

        if(xDiff > 0) {
            setFacing(2);
        } else if(xDiff < 0) {
            setFacing(4);
        }

        moveXTiles(Math.abs(xDiff));
        pRotateNoReset(-.2, 0);

        stopMotors();

    }
    //=============BEGIN CALCULATION METHODS==========================

    public int getEncoderAvg() {
        return ((Math.abs(motorL.getCurrentPosition())) + (Math.abs(motorR.getCurrentPosition())))
                / 2;
    }
    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public void getAngles() {
        angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    public boolean isRightRed() {
        return rightColor.red() > 450;
    }

    public boolean isLeftRed() {
        return leftColor.red() > 450;
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
