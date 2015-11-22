package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 11/21/2015.
 */
public class ServoTest2 extends OpMode {
    Servo rightBar;
    Servo leftBar;
    public void init() {
        rightBar = hardwareMap.servo.get("rightBar");
        leftBar = hardwareMap.servo.get("leftBar");
        rightBar.setPosition(0);
        leftBar.setPosition(0);

    }
    public void loop() {
        telemetry.addData("rightBar", rightBar.getPosition());
        telemetry.addData("leftBar", leftBar.getPosition());
        try {
            wait(500);
        } catch(InterruptedException e) {

        }
    }
    public void stop() {
        rightBar.setPosition(1);
        leftBar.setPosition(1);
    }
}
