//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
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
    public int currentEncoder;
    public int BLencoder;
    public int BRencoder;
    public int FRencoder;
    public int FLencoder;
    public int BRnullEncoder;
    public int BLnullEncoder;
    public int FRnullEncoder;
    public int FLnullEncoder;
    public DeviceInterfaceModule cdim;
    public double currentAngle;
    public TouchSensor rts;
    public TouchSensor lts;
    private boolean hit;
    private static final double UNDROPPED = .55;
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


    public void moveForward(double pow) {
        while(!hit) {
            startMotors(pow, pow);
            if(rts.isPressed() || lts.isPressed()) {
                hit = true;
            }
        }
        stopMotors();
    }

    public void myWait(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            RobotLog.e(e.getMessage());
        }

    }
    public void raiseLifts(double pow, int time) {
        ElapsedTime thisTime = new ElapsedTime();
        thisTime.startTime();
        while (thisTime.time() < time) {
            liftL.setPower(pow);
            liftR.setPower(-pow);
        }
        liftL.setPower(0);
        liftR.setPower(0);
    }

    public void dumpClimbers() {
        climberSwitch.setPosition(1);
    }

    public void resetClimbers() {
        climberSwitch.setPosition(.55);
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
        motorBL.setPower(-le);
        motorBR.setPower(ri);
        motorFL.setPower(-le);
        motorFR.setPower(ri);
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



    public final void first() {
        motorBL = hardwareMap.dcMotor.get("BL");
        manipulator = hardwareMap.dcMotor.get("mani");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        //liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        basket = hardwareMap.servo.get("basketLeft");
//        climberSwitch = hardwareMap.servo.get("switch");
//        rightRatchet = hardwareMap.servo.get("ratchetR");
//        leftRatchet = hardwareMap.servo.get("ratchetL");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");
        rightPaddle.setPosition(RIGHTPADDLE_IN);
        leftPaddle.setPosition(LEFTPADDLE_IN);
//        leftRatchet.setPosition(0);
//        rightRatchet.setPosition(0);
        basket.setPosition(.5);
//        climberSwitch.setPosition(.55);
        BLnullEncoder = 0;
        BRnullEncoder = 0;
        FRencoder = 0;
        FLencoder = 0;
        BRencoder = 0;
        BLencoder = 0;
        FLnullEncoder = 0;
        FRnullEncoder = 0;
        currentEncoder = 0;
        currentAngle = 0;
        hit = false;
        rts = hardwareMap.touchSensor.get("rts");
        lts = hardwareMap.touchSensor.get("lts");
    }

}
