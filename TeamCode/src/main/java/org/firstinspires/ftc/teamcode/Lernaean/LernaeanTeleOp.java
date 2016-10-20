package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    boolean isbackOut = false;
    boolean isfrontOut = false;

    @Override
    public void loop(){

        if(gamepad2.right_bumper) {
            startShooter();
        }

        if(gamepad2.left_bumper)
            stopShooter();

        if(gamepad2.a){
            if (!isbackOut) {
                backOut();
            } else {
                backIn();
            }
        }

        if(gamepad2.x){
            if(!isfrontOut){
                frontOut();
            } else {
                frontIn();
            }
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
            startMotors(gamepad1.right_stick_y,gamepad1.left_stick_y );
        } else {
            stopMotors();
        }

        if(gamepad1.right_bumper) {
            startMani();
        }

        if(gamepad1.left_bumper)
            stopMani();

        if(gamepad1.a) {
            while(gamepad1.a);
            reverse();
        }

        telemetry.update();
    }
}
