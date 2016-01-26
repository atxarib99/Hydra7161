package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import com.qualcomm.ftcrobotcontroller.opmodes.MyOpMode;

/**
 * Created by Arib on 1/4/2016.
 */
public class TeleOP extends MyOpMode {

    int mani = 1;
    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
        } else {
            stopMotors();
        }
        if(gamepad2.right_trigger > .05) {
            raiseRightLift(gamepad2.right_trigger);
        }
        else if(gamepad2.right_bumper) {
            lowerRightLift();
        }
        else {
            stopRightLift();
        }

        if(gamepad2.left_trigger > .05) {
            raiseLeftLift(gamepad2.left_trigger);
        }
        else if(gamepad2.left_bumper) {
            lowerLeftLift();
        }
        else {
            stopLeftLift();
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
        if(gamepad2.right_stick_button) {
            mani = 1;
        }

        if(gamepad2.left_stick_button) {
            mani = 0;
        }
        if(gamepad2.right_stick_y > .95 && gamepad2.left_stick_y > .95) {
            mani = -1;
        }

        if(gamepad2.y) {
            resetClimbers();
        }

//        if(gamepad1.x) {
//            dropRatchets();
//        }
//
//        if(gamepad1.y) {
//            undoRatchets();
//        }

        if(mani == 1) {
            startManipulator();
        } else if(mani == -1) {
            reverseManipulator();
        } else {
            stopManipulator();
        }
        if(gamepad2.b) {
            dump();
        }
        if(gamepad2.x) {
            unDump();
        }


    }

}
