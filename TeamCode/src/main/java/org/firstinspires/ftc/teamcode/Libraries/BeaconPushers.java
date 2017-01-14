package org.firstinspires.ftc.teamcode.Libraries;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;

/**
 * Created by Arib on 10/17/2016.
 */
public class BeaconPushers {
    Servo back;
    Servo front;
    ColorSensor colorR; //0x2c
    ColorSensor colorL; //0x3c

    LinearOpMode opMode;
    private final double BACK_IN = .7;
    private final double BACK_OUT = 0;
    private final double FRONT_IN = .4;
    private final int FRONT_OUT = 0;
    private double BEACON_BLUE = 0.0;

    private final String LOG_TAG = "BeaconPushers";
    public BeaconPushers(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;

        back = this.opMode.hardwareMap.servo.get("back");
        front = this.opMode.hardwareMap.servo.get("front");
        colorR = this.opMode.hardwareMap.colorSensor.get("colorR");
        colorL = this.opMode.hardwareMap.colorSensor.get("colorL");
        colorR.setI2cAddress(I2cAddr.create8bit(0x2c));
        colorL.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorR.enableLed(false);
        colorL.enableLed(false);
        frontOut(false);
        backOut(false);
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }

    public boolean isBackBlue() throws InterruptedException {
        opMode.telemetry.addData("colorR val", colorR.red());
        opMode.telemetry.addData("colorL val", colorL.red());
        opMode.telemetry.update();
        Thread.sleep(1000);

        double bluebeaconL = colorL.blue();
        double bluebeaconR = colorR.blue();
        double redbeaconL = colorL.red();
        double redbeaconR = colorR.red();
        
        BEACON_BLUE += bluebeaconR - bluebeaconL;
        BEACON_BLUE += redbeaconL - redbeaconR;

        return BEACON_BLUE >= 0.0;
    }

    public String getColorVal() {

        String redR = "RightRed: " + colorR.red() + " ";
        String blueR = "RightBlue: " + colorR.blue() + " ";
        String redL = "LeftRed: " + colorL.red() + " ";
        String blueL = "LeftBlue: " + colorL.blue() + " ";
        return redR + blueR + redL + blueL;
    }

    public void frontOut(boolean works) throws InterruptedException {
        if(works) {
            front.setPosition(1);
            Thread.sleep(200);
            front.setPosition(FRONT_OUT);
        }
        else {
            front.setPosition(FRONT_IN);
        }

    }

    public void backOut(boolean works) throws InterruptedException {
        if (works) {
            back.setPosition(1);
            Thread.sleep(200);
            back.setPosition(BACK_OUT);
        }
        else {
            back.setPosition(BACK_IN);
        }

    }

    public void backPush() throws InterruptedException {
        backOut(true);
        Thread.sleep(1000);
        backOut(false);
    }

    public void frontPush() throws InterruptedException {
        frontOut(true);
        Thread.sleep(1000);
        frontOut(false);
    }



}
