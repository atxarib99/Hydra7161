//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import java.util.concurrent.TimeUnit;

public abstract class MyOpMode extends OpMode {
    private static final double UNDROPPED = .55;
    private static final double DROPPED = 1;
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
    public Servo basketLeft;
    public Servo basketRight;
    public MyOpMode() {
        super();
    }

    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        manipulator = hardwareMap.dcMotor.get("mani");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        basket = hardwareMap.servo.get("basketLeft");
        basketRight = hardwareMap.servo.get("bright");
        basketLeft = hardwareMap.servo.get("lright");
        climberSwitch = hardwareMap.servo.get("switch");
//        rightRatchet = hardwareMap.servo.get("ratchetR");
//        leftRatchet = hardwareMap.servo.get("ratchetL");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");
        rightPaddle.setPosition(0);//TODO: UPDATE THESE VALUES LATER
        leftPaddle.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//        leftRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
//        rightRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
        climberSwitch.setPosition(.55);
    }

    public void dumpClimbers() {
        climberSwitch.setPosition(DROPPED);
    }

    public void resetClimbers() {
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

    public void startMotors(double ri, double le) {
        motorBL.setPower(le);
        motorBR.setPower(-ri);
        motorFL.setPower(le);
        motorFR.setPower(-ri);
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
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

    public void raiseLifts(double pow) {
        liftL.setPower(pow);
        liftR.setPower(-pow);
    }

    public void resetEncoders() {
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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

}
