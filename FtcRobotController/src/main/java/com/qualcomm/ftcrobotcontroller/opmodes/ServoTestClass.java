package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 11/21/2015.
 */
public class ServoTestClass extends OpMode {
//    public Servo climberSwitch;
    //    public Servo rightRatchet;
    //    public Servo leftRatchet;
    public Servo rightPaddle;
    public Servo leftPaddle;
    public Servo basket;
    public Servo basketLeft;
    public Servo basketRight;
    public void init() {
        basket = hardwareMap.servo.get("basketLeft");
        basketRight = hardwareMap.servo.get("bright");
        basketLeft = hardwareMap.servo.get("lright");
//        climberSwitch = hardwareMap.servo.get("switch");
//        rightRatchet = hardwareMap.servo.get("ratchetR");
//        leftRatchet = hardwareMap.servo.get("ratchetL");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");
        rightPaddle.setPosition(0);//TODO: UPDATE THESE VALUES LATER
        leftPaddle.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//        leftRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
//        rightRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
        basket.setPosition(.5); //TODO: UPDATE THESE VALUES LATER
        basketLeft.setPosition(1); //TODO: UPDATE THESE VALUES LATER
        basketRight.setPosition(0); //TODO: UPDATE THESE VALUES LATER
//        climberSwitch.setPosition(.55);

    }
    public void loop() {
        if(gamepad1.x) {
//            climberSwitch.setPosition(1);
//            telemetry.addData("climberSwitch", climberSwitch.getPosition());
            basketRight.setPosition(1);
            telemetry.addData("basketRight", basketRight.getPosition());
            basket.setPosition(1);
            telemetry.addData("basket", basket.getPosition());
            basketLeft.setPosition(1);
            telemetry.addData("basketLeft",  basketLeft.getPosition());
            rightPaddle.setPosition(1);
            telemetry.addData("rightPaddle", rightPaddle.getPosition());
            leftPaddle.setPosition(1);
            telemetry.addData("leftPaddle", leftPaddle.getPosition());
        }
        if(gamepad1.b) {
//            climberSwitch.setPosition(0);
//            telemetry.addData("climberSwitch", climberSwitch.getPosition());
            basketRight.setPosition(0);
            telemetry.addData("basketRight", basketRight.getPosition());
            basket.setPosition(0);
            telemetry.addData("basket", basket.getPosition());
            basketLeft.setPosition(0);
            telemetry.addData("basketLeft",  basketLeft.getPosition());
            rightPaddle.setPosition(0);
            telemetry.addData("rightPaddle", rightPaddle.getPosition());
            leftPaddle.setPosition(0);
            telemetry.addData("leftPaddle", leftPaddle.getPosition());
        }

    }
    public void stop() {

    }
}
