package org.firstinspires.ftc.teamcode.TrollBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by TechMaster on 9/22/2016.
 */
@TeleOp(name = "TrollTele-OP", group = "OpMode")
public class TrollTeleOp extends TrollOpMode {

    @Override
        public void loop() {
            if((Math.abs(gamepad1.right_stick_y) > .05) || Math.abs(gamepad1.left_stick_y) > .05){
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
            } else {
            stopMotors();
            }

            getAngles();

            composeTelemetry();

            telemetry.update();
    }
}
