//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import java.util.concurrent.TimeUnit;

public abstract class MyOpMode extends OpMode {
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
    public MyOpMode() {
        super();
    }

    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        manipulator = hardwareMap.dcMotor.get("mani");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rts = hardwareMap.digitalChannel.get("rts");
        lts = hardwareMap.digitalChannel.get("lts");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
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
//        telemetry.addData("gyro", "initializing...");
//
//        try {
//            gyro = new AdafruitIMU(hardwareMap, "hydro"
//
//                    //The following was required when the definition of the "I2cDevice" class was incomplete.
//                    //, "cdim", 5
//
//                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
//                    //addressing
//                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
//        } catch (RobotCoreException e){
//            Log.i("FtcRobotController", "Exception: " + e.getMessage());
//            telemetry.addData("gyro", "fail");
//        }
//
//        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");


    }

    public void dumpClimbers() {
        climberSwitch.setPosition(DROPPED);
   }

    public void resetClimbers() {
        climberSwitch.setPosition(UNDROPPED);
    }

//    public void dropRatchets() {
//        leftRatchet.setPosition(1);
//        rightRatchet.setPosition(1);
//    }
//
//    public void undoRatchets() {
//        leftRatchet.setPosition(1);
//        rightRatchet.setPosition(1);
//
//    }

    public void extendPaddles() {
        rightPaddle.setPosition(RIGHTPADDLE_OUT);
        leftPaddle.setPosition(LEFTPADDLE_OUT);
    }

    public void extendRightPaddle() {
        rightPaddle.setPosition(RIGHTPADDLE_OUT);
    }

    public void retractRightPaddle() {
        rightPaddle.setPosition(RIGHTPADDLE_IN);
    }

    public void extendLeftPaddle() {
        leftPaddle.setPosition(LEFTPADDLE_OUT);
    }

    public void retractLeftPaddle() {
        leftPaddle.setPosition(LEFTPADDLE_IN);
    }

     public void retractPaddles() {
         rightPaddle.setPosition(RIGHTPADDLE_IN);
         leftPaddle.setPosition(LEFTPADDLE_IN);
     }

    public void dump() {
        basket.setPosition(BASKET_DUMPED);
    }


    public void dumpRight() {
        basket.setPosition(BASKET_RIGHT);
    }

    public void dumpleft() {
        basket.setPosition(BASKET_LEFT);
    }

    public void dumpIdle() {
        basket.setPosition(BASKET_IDLE);
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

    public void raiseLifts(double pow) {
        liftL.setPower(pow);
        liftR.setPower(-pow);
    }
    public void raiseRightLift(double pow) {
        liftR.setPower(-pow);
    }
    public void raiseLeftLift(double pow) {
        liftL.setPower(pow);
    }
    public void stopLeftLift() {
        liftL.setPower(0);
    }
    public void stopRightLift() {
        liftR.setPower(0);
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
    public void lowerLeftLift() {
        liftL.setPower(-.5);
    }
    public void lowerRightLift() {
        liftR.setPower(.5);
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
