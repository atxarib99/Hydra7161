//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;
import android.media.MediaPlayer;
import android.test.InstrumentationTestRunner;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public abstract class AutoMode extends LinearOpMode {
    //Adafruit BNO055 gyro
    public AdafruitIMU gyro;

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

    //MediaPlayer object to get the RobotController to play sounds
    public MediaPlayer song;

    //DeviceInterfaveModule for sensors
    public DeviceInterfaceModule cdim;
    public ColorSensor sensorRGB;       //Adafruit color sensor
    public DigitalChannel rts;          //right touch sensor
    public DigitalChannel lts;          //left touch sensor

    //volatile double array to hold gyro angle
    static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    private boolean hit;

    //static final variables to hold servo positions
    private static final double UNDROPPED = 0;
    private static final double DROPPED = 1;
    private static final double RIGHTPADDLE_OUT = .75;
    private static final double LEFTPADDLE_OUT = .5;
    private static final double RIGHTPADDLE_IN = 1;
    private static final double LEFTPADDLE_IN = 0;
    private static final double LEFTDUMPER_DUMPED = 0;
    private static final double LEFTDUMPER_UNDUMPED = 1;
    private static final double RIGHTDUMPER_DUMPED = 1;
    private static final double RIGHTDUMPER_UNDUMPED = 0;
    private static final double BASKET_LEFT = 0;
    private static final double BASKET_RIGHT = 1;
    private static final double BASKET_IDLE = .5;

    //constructor
    public AutoMode() {
        super(); //ADDED ON 3/1/2016
    }

    //This method moves forward at a given power to a given encoder value
    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        //notifies Driver Station of current event
        telemetry.addData("Auto", "Moving Forwards");

        //resets the Gyro's angle
        resetGyro();
        double angle;
        sendData();

        //while target is not reached
        while(!hit && (encoderVal > getBackWheelAvg())) {
            waitOneFullHardwareCycle();
            telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
            telemetry.addData("avg", getBackWheelAvg());
            sendData();
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            angle = yawAngle[0];

            //if off to the left, correct
            if(angle > 2) {
                startMotors(pow, (pow / 2));
                sendData();
                waitOneFullHardwareCycle();
            } else if(angle < -2) { //if off to the right, correct
                startMotors((pow / 2), pow);
                waitOneFullHardwareCycle();
                sendData();
            } else { //if heading is fine keep moving straight
                startMotors(pow, pow);
                sendData();
                waitOneFullHardwareCycle();
            }

            //Check if touch sensors are hit
            if(rts.getState() || lts.getState()) {
                hit = true;
            }
        }

        //once finished stop moving and send data
        waitOneFullHardwareCycle();
        stopMotors();
        sendData();
        waitOneFullHardwareCycle();
    }

    //raise lifts for a certain amount of time due to lack of encoders.
    public void raiseLifts(double pow, int time) throws InterruptedException {
        ElapsedTime thisTime = new ElapsedTime();
        thisTime.startTime();
        waitOneFullHardwareCycle();
        while (thisTime.time() < time) {
            liftL.setPower(pow);
            liftR.setPower(-pow);
            waitOneFullHardwareCycle();
        }
        waitOneFullHardwareCycle();
        liftL.setPower(0);
        liftR.setPower(0);
        waitOneFullHardwareCycle();
    }

    //dispense climbers into shelter
    public void dumpClimbers() throws InterruptedException {
        waitOneFullHardwareCycle();
        climberSwitch.setPosition(DROPPED);
    }

    //reset the climber bar
    public void resetClimbers() throws InterruptedException {
        waitOneFullHardwareCycle();
        climberSwitch.setPosition(UNDROPPED);
    }

    //reset the gyro and its angles
    public void resetGyro() throws InterruptedException {
        gyro.startIMU();
        waitOneFullHardwareCycle();
    }

    //extend both paddles
    public void extendPaddles() {
        rightPaddle.setPosition(RIGHTPADDLE_OUT);
        leftPaddle.setPosition(LEFTPADDLE_OUT);
    }
    public void retractPaddles() {
        rightPaddle.setPosition(RIGHTPADDLE_IN);
        leftPaddle.setPosition(LEFTPADDLE_IN);
    }

    //start the motors in a tank drive
    public void startMotors(double ri, double le) throws InterruptedException {
        motorBL.setPower(le);
        waitOneFullHardwareCycle();
        motorBR.setPower(-ri);
        waitOneFullHardwareCycle();
        motorFL.setPower(le);
        waitOneFullHardwareCycle();
        motorFR.setPower(-ri);
        waitOneFullHardwareCycle();
    }

    //stop all the motors
    public void stopMotors() throws InterruptedException {
        waitOneFullHardwareCycle();
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        waitOneFullHardwareCycle();
    }

    //rotate the robot //TODO: ADD PID CONTROL
    public void rotate() throws InterruptedException {
        waitOneFullHardwareCycle();
        resetGyro();
        sendData();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double angle = yawAngle[0];
        //while current angle is greater than desired angle
        while(angle > -15) {
            sendData();
            waitOneFullHardwareCycle();
            startMotors(.2, .2);
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();
        //if overshot
        while(angle < -20) {
            waitOneFullHardwareCycle();
            startMotors(-.2, -.2);
            sendData();
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();
        //if undershot
        while(angle > -10) {
            waitOneFullHardwareCycle();
            startMotors(.2, .2);
            sendData();
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();
        sendData();

    }

    //P loop for Pid
    public void pRotateLeft(double pow, double angle) throws InterruptedException {
        resetGyro();
        double power = pow;
        double angleTo = angle;
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double currentAngle = yawAngle[0];
        while (currentAngle < )
    }
    //start the manipulator
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

    //reset the encoders
    public void resetEncoders() throws InterruptedException {
        while(Math.abs(motorBL.getCurrentPosition()) > 25 || Math.abs(motorBR.getCurrentPosition()) > 25) {
            motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            waitOneFullHardwareCycle();
        }
        waitOneFullHardwareCycle();
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
    }

    //lower the lifts
    public void lowerLifts(double pow) {
        liftL.setPower(-pow);
        liftR.setPower(pow);
    }

    //stops the lifts
    public void stopLifts() {
        liftL.setPower(0);
        liftR.setPower(0);
    }

    //get the encoder average of all the wheels
    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))
                + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition())))
                / 4;
    }

    //get the encoder average of the back wheels
    public int getBackWheelAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())))
                / 2;
    }

    //check to be sure if pitch is ok
    public boolean isOk() throws InterruptedException {
        waitOneFullHardwareCycle();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        waitOneFullHardwareCycle();
        double angle = pitchAngle[0];
        if(angle > .5) {
            return false;
        }
        return true;
    }

    //check to see if heading and pitch are ok after rotation
    public boolean allIsOk() throws InterruptedException {
        waitOneFullHardwareCycle();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        waitOneFullHardwareCycle();
        double rotate = yawAngle[0];
        double pitch = pitchAngle[0];
        if((rotate < 17 && rotate > 13) && pitch < 1) {
            return true;
        }
        return false;
    }

    //get and telemetry data to Driver Station
    public void sendData() {
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

    //Hardware map and initialize motors and servos
    public final void first() {
        motorBL = hardwareMap.dcMotor.get("BL");
        manipulator = hardwareMap.dcMotor.get("mani");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        basket = hardwareMap.servo.get("basket");
        climberSwitch = hardwareMap.servo.get("switch");
//        rightRatchet = hardwareMap.servo.get("ratchetR");
//        leftRatchet = hardwareMap.servo.get("ratchetL");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");
        rightPaddle.setPosition(RIGHTPADDLE_IN);
        leftPaddle.setPosition(LEFTPADDLE_IN);
//        leftRatchet.setPosition(0);
//        rightRatchet.setPosition(0);
        climberSwitch.setPosition(UNDROPPED);
        basket.setPosition(BASKET_IDLE);
        hit = false;
        rts = hardwareMap.digitalChannel.get("rts");
        lts = hardwareMap.digitalChannel.get("lts");
        sensorRGB = hardwareMap.colorSensor.get("color");
        song = MediaPlayer.create(FtcRobotControllerActivity.getContext(), R.raw.move);
        song.setLooping(true);
        song.seekTo(5000);
        song.start();
        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
            telemetry.addData("gyro", "fail");
        }

        hardwareMap.logDevices();


        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");
        telemetry.addData("Auto", "Initialized Successfully!");
    }


}
