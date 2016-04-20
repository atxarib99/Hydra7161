package com.qualcomm.ftcrobotcontroller.opmodes.PinheadLarry;

/**
 * Created by Arib on 4/11/2016.
 */

import com.qualcomm.ftcrobotcontroller.opmodes.PinheadOpMode;

public class PinheadTeleOp extends PinheadOpMode {

    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            motorBL.setPower(-gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
        }
        else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }
    }

}
