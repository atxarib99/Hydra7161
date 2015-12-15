package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by Joshua on 11/12/2015.
 */
public class QualifierTeleOp extends AutoMode {

    @Override
    public void runOpMode() {
        first();
        while(opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
                motorBL.setPower(gamepad1.right_stick_y);
                motorBR.setPower(-gamepad1.left_stick_y);
                motorFR.setPower(-gamepad1.left_stick_y);
                motorFL.setPower(gamepad1.right_stick_y);
            }
            else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }


        }
    }
}
