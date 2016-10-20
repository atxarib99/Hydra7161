package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 10/17/2016.
 */
public class BeaconPushers {
    Servo back;
    Servo front;
    ColorSensor color;

    LinearOpMode opMode;
    private final int BACK_IN = 0;
    private final int BACK_OUT = 1;
    private final int FRONT_IN = 0;
    private final int FRONT_OUT = 1;

    public BeaconPushers(LinearOpMode opMode) {
        this.opMode = opMode;

        back = this.opMode.hardwareMap.servo.get("back");
        front = this.opMode.hardwareMap.servo.get("front");
        color = this.opMode.hardwareMap.colorSensor.get("color");
    }

    public boolean isBackRed() {
        opMode.telemetry.addData("color val", color.red());
        return color.red() > 450;
    }

    public void frontOut(boolean works) {
        if(works)
            front.setPosition(FRONT_OUT);
        else
            front.setPosition(FRONT_IN);

    }

    public void backOut(boolean works) {
        if (works)
            back.setPosition(BACK_OUT);
        else
            back.setPosition(BACK_IN);

    }



}
