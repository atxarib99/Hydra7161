package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 8/29/2016.
 */

//4 motors
    //servo
public class BasicOpMode extends OpMode {
    DcMotor motorBL;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    Servo arm;
    Servo leg;
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        arm = hardwareMap.servo.get("arm");
        leg = hardwareMap.servo.get("leg");
        motorBL.setPower(0);
        arm.setPosition(0);
    }

    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
            motorBL.setPower(-gamepad1.left_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
        } else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
        }

        if(gamepad1.a){
            arm.setPosition(.5);
        }

        if(gamepad1.b) {
            leg.setPosition(1);
        }

    }

    @Override
    public void stop() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        leg.setPosition(.5);
    }

}
