package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by Arib on 9/27/2015.
 */
public class ColorTeleOp extends OpMode {
    DcMotor motorBL;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    ColorSensor colors;
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        colors = hardwareMap.colorSensor.get("color");
    }
    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y ) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
            motorBL.setPower(gamepad1.left_stick_y);
            motorFL.setPower(gamepad1.left_stick_y);
            motorFR.setPower(-gamepad1.right_stick_y);
            motorBR.setPower(-gamepad1.right_stick_y);
            colors.enableLed(false);
        }
        else {
            motorBL.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            colors.enableLed(true);
        }
        if(gamepad1.x) {
            telemetry.addData("blue", colors.blue());
            telemetry.addData("red", colors.blue());
            telemetry.addData("green", colors.green());
            telemetry.addData("hue", colors.argb());
            telemetry.addData("Light", colors.alpha());
        }

    }
    @Override
    public void stop() {

    }

}
