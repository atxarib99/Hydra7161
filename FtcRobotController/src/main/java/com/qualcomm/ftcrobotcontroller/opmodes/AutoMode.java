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
    public AdafruitIMU gyro;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor manipulator;
    public DcMotor liftL;
    public DcMotor liftR;
    public Servo climberSwitch;
    //    public Servo rightRatchet;
//    public Servo leftRatchet;
    public Servo rightPaddle;
    public Servo leftPaddle;
    public MediaPlayer song;
    public Servo basket;
    public DeviceInterfaceModule cdim;
    public ColorSensor color;
    public DigitalChannel rts;
    public DigitalChannel lts;
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    private boolean hit;
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

    public AutoMode() {

    }


    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        telemetry.addData("Auto", "Moving Forwards");
        resetGyro();
        double angle;
        while(!hit && (encoderVal > getBackWheelAvg())) {
            waitOneFullHardwareCycle();
            telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
            telemetry.addData("avg", getBackWheelAvg());
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            angle = yawAngle[0];
            if(angle > 2) {
                startMotors(pow, (pow / 2));
                waitOneFullHardwareCycle();
            }
            else if(angle < -2) {
                startMotors((pow / 2), pow);
                waitOneFullHardwareCycle();
            }
            else {
                startMotors(pow, pow);
                waitOneFullHardwareCycle();
            }
            if(rts.getState() || lts.getState()) {
                hit = true;
            }
        }
        waitOneFullHardwareCycle();
        stopMotors();
        waitOneFullHardwareCycle();
    }

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

    public void dumpClimbers() throws InterruptedException {
        waitOneFullHardwareCycle();
        climberSwitch.setPosition(DROPPED);
    }

    public void resetClimbers() throws InterruptedException {
        waitOneFullHardwareCycle();
        climberSwitch.setPosition(UNDROPPED);
    }

    public void resetGyro() throws InterruptedException {
        gyro.startIMU();
        waitOneFullHardwareCycle();
    }

//    public void dropRatchets() {
//        leftRatchet.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//        rightRatchet.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//    }
//
//    public void undoRatchets() {
//        leftRatchet.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//        rightRatchet.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//
//    }

    public void extendPaddles() {
        rightPaddle.setPosition(1); //TODO: UPDATE THESE VALUES LATER
        leftPaddle.setPosition(0); //TODO: UPDATE THESE VALUES LATER
    }
    public void retractPaddles() {
        rightPaddle.setPosition(0); //TODO: UPDATE THESE VALUES LATER
        leftPaddle.setPosition(1); //TODO: UPDATE THESE VALUES LATER
    }

    public void startMotors(double ri, double le) throws InterruptedException {
        motorBL.setPower(le);
        motorBR.setPower(-ri);
        motorFL.setPower(le);
        motorFR.setPower(-ri);
    }

    public void stopMotors() throws InterruptedException {
        waitOneFullHardwareCycle();
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        waitOneFullHardwareCycle();
    }

    public void rotate() throws InterruptedException {
        waitOneFullHardwareCycle();
        resetGyro();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double angle = yawAngle[0];
        while(angle > -15) {
            waitOneFullHardwareCycle();
            startMotors(.2, .2);
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();
        while(angle < -20) {
            waitOneFullHardwareCycle();
            startMotors(-.2, -.2);
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();
        while(angle > -10) {
            waitOneFullHardwareCycle();
            startMotors(.2, .2);
            waitOneFullHardwareCycle();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            waitOneFullHardwareCycle();
            angle = yawAngle[0];
        }
        stopMotors();

    }

    public void startManipulator() {
        manipulator.setPower(1);
    }

    public void stopManipulator() {
        manipulator.setPower(0);
    }

    public void reverseManipulator() {
        manipulator.setPower(-1);
    }

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

    public void lowerLifts(double pow) {
        liftL.setPower(-pow);
        liftR.setPower(pow);
    }

    public void stopLifts() {
        liftL.setPower(0);
        liftR.setPower(0);
    }

    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))
                + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition())))
                / 4;
    }

    public int getBackWheelAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())))
                / 2;
    }

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
        color = hardwareMap.colorSensor.get("color");
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


        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");
        telemetry.addData("Auto", "Initialized Successfully!");
    }


}
