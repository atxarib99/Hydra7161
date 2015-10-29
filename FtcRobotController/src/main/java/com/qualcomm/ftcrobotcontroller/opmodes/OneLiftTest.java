package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 10/29/2015.
 */
public class OneLiftTest extends OpMode {

    DcMotor lift;

    public void init() {
        lift = hardwareMap.dcMotor.get("lift");

    }
    public void loop (){
        if(Math.abs(gamepad1.right_stick_y) > .05) {
            lift.setPower(gamepad1.right_stick_y);
        }
        else
            lift.setPower(0);
    }
    public void stop() {

    }
}
