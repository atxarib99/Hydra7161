//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;
import android.test.InstrumentationTestRunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
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
    public Servo basket;
    public DeviceInterfaceModule cdim;
    public DigitalChannel rts;
    public DigitalChannel lts;
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
    private static final double BASKET_IDLE = .35;

    public AutoMode() {

    }


    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        while(!hit && (encoderVal > getBackWheelAvg())) {
            waitOneFullHardwareCycle();
            startMotors(pow, pow);
            telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
            telemetry.addData("avg", getBackWheelAvg());
            waitOneFullHardwareCycle();
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
        while(motorBL.getCurrentPosition() > 25 || motorBR.getCurrentPosition() > 25) {
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
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())) + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition()))) / 4;
    }

    public int getBackWheelAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))) / 2;
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
        basket.setPosition(BASKET_IDLE);
        climberSwitch.setPosition(UNDROPPED);
        hit = false;
        rts = hardwareMap.digitalChannel.get("rts");
        lts = hardwareMap.digitalChannel.get("lts");
    }


}
