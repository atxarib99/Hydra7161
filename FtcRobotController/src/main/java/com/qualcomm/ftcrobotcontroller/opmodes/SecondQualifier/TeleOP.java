package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import com.qualcomm.ftcrobotcontroller.opmodes.MyOpMode;

/**
 * Created by Arib on 1/4/2016.
 */
public class TeleOP extends MyOpMode {

    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
        } else {
            stopMotors();
        }

        if(gamepad2.right_trigger > .05) {
            raiseLifts(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > .05) {
            lowerLifts(gamepad2.left_trigger);
        }
        else {
            stopLifts();
        }

        if(gamepad1.right_bumper) {
            extendPaddles();
        }

        if(gamepad1.left_bumper) {
            retractPaddles();
        }

        if(gamepad2.a) {
            dumpClimbers();
        }

        if(gamepad2.b) {
            resetClimbers();
        }

        if(gamepad2.x) {
            dropRatchets();
        }

        if(gamepad2.y) {
            undoRatchets();
        }

        if(gamepad2.right_bumper) {
            startManipulator();
        } else if(gamepad2.left_bumper) {
            reverseManipulator();
        } else {
            stopManipulator();
        }


    }

}
