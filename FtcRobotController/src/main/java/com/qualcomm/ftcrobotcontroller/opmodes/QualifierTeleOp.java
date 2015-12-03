package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by Joshua on 11/12/2015.
 */
public class QualifierTeleOp extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    Servo rightBar;
    Boolean frontWheels;
    Servo leftBar;
    private static final String LOG_TAG = QualifierTeleOp.class.getSimpleName();
    private int getEncoderAvg() {
        return (motorBL.getCurrentPosition() + motorBR.getCurrentPosition() + motorFL.getCurrentPosition() + motorFR.getCurrentPosition()) / 4;
    }
    public void extendBars() {
        rightBar.setPosition(.95);
        leftBar.setPosition(0);
    }
    public void contractBars() {
        rightBar.setPosition(0);
        leftBar.setPosition(.9);
    }

    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        rightBar = hardwareMap.servo.get("rightBar");
        frontWheels = false;
        leftBar = hardwareMap.servo.get("leftBar");

    }
    public void loop() {
        if ((Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) && frontWheels == true) {
            motorBL.setPower(gamepad1.right_stick_y);
            motorFL.setPower(gamepad1.right_stick_y);
            motorBR.setPower(-gamepad1.left_stick_y);
            motorFR.setPower(-gamepad1.left_stick_y);
        } else if((Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) && frontWheels == false) {
            motorBL.setPower(gamepad1.right_stick_y);
            motorBR.setPower(-gamepad1.left_stick_y);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }
        else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
            
        }

        telemetry.addData("motorBL", motorBL.getCurrentPosition());

        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("avg", getEncoderAvg());
        if (gamepad1.a) {
            motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorBL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorBR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorFL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorFR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        if (gamepad1.x) {
            frontWheels = true;
        }
        if(gamepad1.y) {
            frontWheels = false;
        }
        //extends side bars to release climbers
        if(gamepad1.right_bumper)
            extendBars();
            //contracts side bars to pull back bars
        if(gamepad1.left_bumper)
            contractBars();
    }
    public void stop() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
    }
}
