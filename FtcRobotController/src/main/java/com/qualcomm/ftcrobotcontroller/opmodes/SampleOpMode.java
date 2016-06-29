package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 6/28/2016.
 */
public class SampleOpMode extends OpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    Servo climbers;
    Servo paddles;
    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        climbers = hardwareMap.servo.get("climbers");
        paddles = hardwareMap.servo.get("paddles");

        climbers.setPosition(0);
        paddles.setPosition(1);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }

    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_x) > .05) {
            motorBL.setPower(-gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
        } else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }

        if(gamepad2.a) {
            if(climbers.getPosition() == 1) {
                climbers.setPosition(0);
            } else {
                climbers.setPosition(1);
            }
        }

        if(gamepad2.x) {
            if(paddles.getPosition() == 1) {
                paddles.setPosition(0);
            } else {
                paddles.setPosition(1);
            }
        }
    }

    public void stop() {
        climbers.setPosition(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
    }
}
