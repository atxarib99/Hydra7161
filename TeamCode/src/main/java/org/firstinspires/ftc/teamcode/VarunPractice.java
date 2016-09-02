package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 8/31/2016.
 */
public class VarunPractice extends OpMode {
    DcMotor MotorBL;
    DcMotor MotorBR;
    DcMotor MotorFL;
    DcMotor MotorFR;
    Servo arm;
    @Override
    public void init() {
        MotorBL = hardwareMap.dcMotor.get("BL");
        MotorBR = hardwareMap.dcMotor.get("BR");
        MotorFL = hardwareMap.dcMotor.get("FL");
        MotorFR = hardwareMap.dcMotor.get("FR");
        arm = hardwareMap.servo.get("arm");

    }

    public void loop(){
        if (Math.abs(gamepad1.right_stick_y) > .05){

        }
    }



}
