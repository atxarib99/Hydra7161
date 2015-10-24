package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 10/24/2015.
 */
public class justTheMotors extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFR;
    DcMotor motorFL;
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
    }
    @Override
    public void loop() {
        telemetry.addData("motorBL", motorBL.getCurrentPosition());
        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());

        if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
            motorBL.setPower(-gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
        }
        else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
        }
    }
    @Override
    public void stop() {

    }
}
