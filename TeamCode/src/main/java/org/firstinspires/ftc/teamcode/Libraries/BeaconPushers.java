package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 10/17/2016.
 */
public class BeaconPushers {
    Servo back;
    Servo front;
    ColorSensor color;

    LinearOpMode opMode;
    private final int BACK_IN = 1;
    private final int BACK_OUT = 0;
    private final int FRONT_IN = 1;
    private final int FRONT_OUT = 0;

    private final String LOG_TAG = "BeaconPushers";
    public BeaconPushers(LinearOpMode opMode) {
        this.opMode = opMode;

        back = this.opMode.hardwareMap.servo.get("back");
        front = this.opMode.hardwareMap.servo.get("front");
        color = this.opMode.hardwareMap.colorSensor.get("color");
        frontOut(false);
        backOut(false);
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }

    public boolean isBackRed() throws InterruptedException {
        opMode.telemetry.addData("color val", color.red());
        opMode.telemetry.update();
        Thread.sleep(2000);
        return color.red() > 250;
    }

    public int getColorVal() {

        return color.red();
    }

    public void frontOut(boolean works) {
        if(works) {
            front.setPosition(FRONT_OUT);
        }
        else {
            front.setPosition(FRONT_IN);
        }

    }

    public void backOut(boolean works) {
        if (works) {
            back.setPosition(BACK_OUT);
        }
        else {
            back.setPosition(BACK_IN);
        }

    }

    public void backPush() throws InterruptedException {
        backOut(true);
        Thread.sleep(2000);
        backOut(false);
    }

    public void frontPush() throws InterruptedException {
        frontOut(true);
        Thread.sleep(2000);
        frontOut(false);
    }



}
