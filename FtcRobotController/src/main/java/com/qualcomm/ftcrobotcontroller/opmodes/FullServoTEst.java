package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 11/3/2015.
 */
public class FullServoTEst extends OpMode {
    Servo basket;
    int nothing;
    public void init(){
        basket = hardwareMap.servo.get("servo");
    }
    public void loop(){

        if(gamepad1.right_bumper) {
            if (basket.getPosition() == Servo.MAX_POSITION) {
                basket.setPosition(Servo.MIN_POSITION);
            }
            basket.setPosition(basket.getPosition() + 1);
        }
        if(gamepad1.left_bumper) {
            if (basket.getPosition() <= Servo.MIN_POSITION) {
                basket.setPosition(Servo.MAX_POSITION);
            }
            basket.setPosition(basket.getPosition() - 1);
        }
        //moves basket
    }
    public void stop(){
        basket.setPosition(Servo.MIN_POSITION);
    }
}
