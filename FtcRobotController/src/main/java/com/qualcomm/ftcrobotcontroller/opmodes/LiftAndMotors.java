package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 11/21/2015.
 */
public class LiftAndMotors extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor rightMotor;
    DcMotor leftMotor;
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        rightMotor = hardwareMap.dcMotor.get("rightClaw");
        leftMotor = hardwareMap.dcMotor.get("leftClaw");
    }
    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            motorBL.setPower(-gamepad1.left_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
        }
        else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }
        if(gamepad1.right_trigger > .05) {
            leftMotor.setPower(1);
            rightMotor.setPower(-1);
        }
        else if(gamepad2.left_trigger > .05) {
            leftMotor.setPower(-1);
            rightMotor.setPower(1);
        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    @Override
    public void stop() {

    }
}
