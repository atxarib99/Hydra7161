//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.OldFiles;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class MyOpMode extends OpMode {
    //static finals variables to hold servo values.
    private static final double UNDROPPED = 0;
    private static final double DROPPED = 1;
    private static final double RIGHTPADDLE_OUT = 0;
    private static final double LEFTPADDLE_OUT = 1;
    private static final double RIGHTPADDLE_IN = 1;
    private static final double LEFTPADDLE_IN = 0;
    private static final double BASKET_DUMPED = .9;
    private static final double BASKET_IDLE = .5;
    private static final double BASKET_LEFT = 1;
    private static final double BASKET_RIGHT = 0;
    //final String for logging
    private static final String LOG_TAG = MyOpMode.class.getSimpleName();
    //volatile double to store gyro values
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    //boolean to hold value if basket is running
    public boolean running;
    //The AdafruitIMU BNO055 gyro
    public AdafruitIMU gyro;
    //Adafruit colorsensor
    public ColorSensor sensorRGB;
    //motors
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor manipulator;
    public DcMotor liftL;
    public DcMotor liftR;
    //servos
    public Servo climberSwitch;
    public Servo rightPaddle;
    public Servo leftPaddle;
    public Servo basket;
    //Device Interface Module for sensors
    public DeviceInterfaceModule cdim;
    //Two Touch sensors
    public DigitalChannel rts;
    public DigitalChannel lts;
    //constructor
    public MyOpMode() {
        super();
    }

    @Override
    public void init() {
        //hardware mapping
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        manipulator = hardwareMap.dcMotor.get("mani");

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rts = hardwareMap.digitalChannel.get("rts");
        lts = hardwareMap.digitalChannel.get("lts");
        sensorRGB = hardwareMap.colorSensor.get("color");

        basket = hardwareMap.servo.get("basket");
        running = false;
        climberSwitch = hardwareMap.servo.get("switch");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");

        //hardware mapping the more complex things
        telemetry.addData("gyro", "initializing...");

        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
            try {
                Thread.sleep(5000);
            } catch(InterruptedException e) {
                Log.i(LOG_TAG, e.getMessage());
            }

            gyro.startIMU();
        } catch (RobotCoreException e){
            Log.i(LOG_TAG, e.getMessage());
            telemetry.addData("gyro", "fail");
        }
        //logging devices this is REQUIRED for color sensor
        hardwareMap.logDevices();
        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");
        telemetry.addData("init", "pass");

    }

    @Override
    public void start() {
        //starting the gyro
        gyro.startIMU();
        //getting the robot into starting position
        leftPaddle.setPosition(LEFTPADDLE_IN);
        rightPaddle.setPosition(RIGHTPADDLE_IN);
        basket.setPosition(BASKET_IDLE);
        climberSwitch.setPosition(UNDROPPED);
    }

    //dumping the climbers behind the shelter
    public void dumpClimbers() {
        climberSwitch.setPosition(DROPPED);
   }

    //bringing the dumper for the climbers back in to prevent casualties
    public void resetClimbers() {
        climberSwitch.setPosition(UNDROPPED);
    }

    //extends both zip line paddles
    public void extendPaddles() {
        rightPaddle.setPosition(RIGHTPADDLE_OUT);
        leftPaddle.setPosition(LEFTPADDLE_OUT);
    }

    //extend the right zip line paddle
    public void extendRightPaddle() {
        rightPaddle.setPosition(RIGHTPADDLE_OUT);
    }

    //retract the right zip line paddle
    public void retractRightPaddle() {
        rightPaddle.setPosition(RIGHTPADDLE_IN);
    }

    //extend the left zip line paddle
    public void extendLeftPaddle() {
        leftPaddle.setPosition(LEFTPADDLE_OUT);
    }

    //retract the left zip line paddle
    public void retractLeftPaddle() {
        leftPaddle.setPosition(LEFTPADDLE_IN);
    }

    //retract both paddles
     public void retractPaddles() {
         rightPaddle.setPosition(RIGHTPADDLE_IN);
         leftPaddle.setPosition(LEFTPADDLE_IN);
     }

    //dump the blocks in the basket NO LONGER SUPPORTED BY NEW DESIGN
    public void dump() {
        basket.setPosition(BASKET_DUMPED);
    }

    //dump the blocks to the right or stop the basket if it is already running
    public void dumpRight() {
        if(running) {
            dumpIdle();
            running = false;
        }
        else {
            basket.setPosition(BASKET_RIGHT);
            running = true;
        }
    }

    //dump the blocks to the left or stop the basket if it is already running
    public void dumpleft() {
        if(running) {
            dumpIdle();
            running = false;
        }
        else {
            basket.setPosition(BASKET_LEFT);

            running = true;
        }
    }

    //stop the basket from moving
    public void dumpIdle() {
        basket.setPosition(BASKET_IDLE);
    }

    //starts the base drive motors in a tank drive
    public void startMotors(double ri, double le) {
        motorBL.setPower(-le);
        motorBR.setPower(ri);
        motorFL.setPower(-le);
        motorFR.setPower(ri);
    }

    //stops all the base drive motors
    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
    }

    //start the manipulators
    public void startManipulator() {
        manipulator.setPower(1);
    }

    //stop the manipulator
    public void stopManipulator() {
        manipulator.setPower(0);
    }

    //reverse the manipulator
    public void reverseManipulator() {
        manipulator.setPower(-1);
    }

    //raise both lifts together
    public void raiseLifts(double pow) {
        liftL.setPower(pow);
        liftR.setPower(-pow);
    }

    //Get values and send them to the driver station
    public void sendData() {
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
        telemetry.addData("Max I2C read interval: ",
                String.format("%4.4f ms. Average interval: %4.4f ms.", gyro.maxReadInterval
                        , gyro.avgReadInterval));
        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());


    }

    //raise only the right lift
    public void raiseRightLift(double pow) {
        liftR.setPower(-pow);
    }

    //this method is supposed to be continually called to find the white line
    public boolean findWhileLine() {
        if(sensorRGB.red() > 1000 && sensorRGB.green() > 1000 && sensorRGB.blue() > 1000) {
            return true;
        }
        return false;
    }
    //raise only the left lift
    public void raiseLeftLift(double pow) {
        liftL.setPower(pow);
    }

    //stops the left lifts movement
    public void stopLeftLift() {
        liftL.setPower(0);
    }

    //stops the right lifts movement
    public void stopRightLift() {
        liftR.setPower(0);
    }

    //lowers both lifts simultaneously
    public void lowerLifts(double pow) {
        liftL.setPower(-pow);
        liftR.setPower(pow);
    }

    //lowers the left lift
    public void lowerLeftLift() {
        liftL.setPower(-.75);
    }

    //lowers the right lift
    public void lowerRightLift() {
        liftR.setPower(.75);
    }

    //stops the lifts
    public void stopLifts() {
        liftL.setPower(0);
        liftR.setPower(0);
    }

    //gets the encoder average of all the base motors
    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())) + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition()))) / 4;
    }

    //gets the encoder average of the back two base motors
    public int getBackWheelAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))) / 2;
    }

}
