package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 11/7/2015.
 */
public class ScrimmageAuto extends LinearOpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    //    DcMotor lift;
    DcMotor rightClaw;
    DcMotor leftClaw;
    ElapsedTime time;
    ElapsedTime fullTime;
    double currTime = 0.0;
    public void runOpMode() {
        //        lift = hardwareMap.dcMotor.get("lift");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        //      basket = hardwareMap.servo.get("basket");
        rightClaw = hardwareMap.dcMotor.get("rightClaw");
        leftClaw = hardwareMap.dcMotor.get("leftClaw");
        try {
            waitForStart();
        } catch (InterruptedException e) {
            telemetry.addData("INTERRUPTED EXCEPTION", e);
        }
        fullTime  = new ElapsedTime();
        fullTime.startTime();
        double fullTimeCount = fullTime.time();
        while (fullTimeCount < 30) {
            time = new ElapsedTime();
            time.startTime();
            currTime = time.time();
            while (currTime < 5) {
                motorBL.setPower(1);
                motorBR.setPower(-.75);
                motorFL.setPower(1);
                motorFR.setPower(-.75);
                currTime = time.time();
                fullTimeCount = fullTime.time();
            }
            fullTimeCount = fullTime.time();
        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
}
