package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import com.qualcomm.ftcrobotcontroller.opmodes.MyOpMode;

/**
 * Created by Arib on 1/4/2016.
 */
public class TeleOP extends MyOpMode {

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    int mani = 0;
    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
        } else {
            stopMotors();
        }

        if(gamepad2.left_trigger > .05) {
            raiseLeftLift(gamepad2.left_trigger);
        } else if(gamepad2.left_bumper) {
            lowerLeftLift();
        } else {
            stopLeftLift();
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
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
        telemetry.addData("Max I2C read interval: ",
                String.format("%4.4f ms. Average interval: %4.4f ms.", gyro.maxReadInterval
                        , gyro.avgReadInterval));
        telemetry.addData("encoderAvg", getBackWheelAvg());
        telemetry.addData("encoderBL", motorBL.getCurrentPosition());
        telemetry.addData("encoderBR", motorBR.getCurrentPosition());

        telemetry.addData("Clear", color.alpha());
        telemetry.addData("Red  ", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue ", color.blue());

        if(gamepad1.right_bumper) {
            extendRightPaddle();
        }

        if(gamepad1.right_trigger > .05) {
            retractRightPaddle();
        }

        if(gamepad1.left_bumper) {
            extendLeftPaddle();
        }

        if(gamepad1.left_trigger > .05) {
            retractLeftPaddle();
        }

        if(gamepad2.a) {
            dumpClimbers();
        }
        if(gamepad1.a) {
            mani = 1;
        }

        if(gamepad1.b) {
            mani = 0;
        }
        if(gamepad1.x) {
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

        if(gamepad2.b && gamepad2.x) {
            dumpIdle();
        }
        else if(gamepad2.b) {
            dumpRight();
        }
        else if(gamepad2.x) { //172.17.1.8
            dumpleft();
        }

        telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
        telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
        telemetry.addData("avg", getBackWheelAvg());


    }

}
