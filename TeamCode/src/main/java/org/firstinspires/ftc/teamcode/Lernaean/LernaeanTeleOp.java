package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    boolean isbackOut = false;
    boolean isfrontOut = false;
    boolean shootMode = true;

    @Override
    public void loop()
    {
        if(shootMode) {

            if(gamepad2.back) {
                while (gamepad2.back) ;
                shootMode = !shootMode;
            }

            if(gamepad1.right_bumper){
                while(gamepad1.right_bumper);
                if (!isbackOut) {
                    backOut();
                    isbackOut = true;
                } else {
                    backIn();
                    isbackOut = false;
                }
            }

            if(gamepad1.left_bumper){
                while(gamepad1.left_bumper);
                if(!isfrontOut){
                    frontOut();
                    isfrontOut = true;
                } else {
                    frontIn();
                    isfrontOut = false;
                }
            }

            if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
                startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
            } else {
                stopMotors();
            }

            if(gamepad1.a) {
                while(gamepad1.a);
                reverse();
            }

            if(gamepad2.right_bumper)
                startShooter();

            if(gamepad2.left_bumper)
                stopShooter();

            if(gamepad2.x) {
                activateShooter(true);
            }

            if(gamepad2.b) {
                activateShooter(false);
            }

            if(gamepad2.dpad_up) {
                while(gamepad2.dpad_up);
                shooterPower += .05;
            }

            if(gamepad2.dpad_down) {
                while(gamepad2.dpad_down);
                shooterPower -= .05;
            }

            if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
                reverseMani();
            } else if(gamepad2.right_trigger > .05) {
                startMani();
            } else if(gamepad2.left_trigger > .05)
                stopMani();
        }

        if(!shootMode) {
            if (gamepad2.back){
                while (gamepad2.back);
                shootMode = !shootMode;
            }

            if(gamepad1.right_bumper){
                while(gamepad1.right_bumper);
                if (!isbackOut) {
                    backOut();
                    isbackOut = true;
                } else {
                    backIn();
                    isbackOut = false;
                }
            }

            if(gamepad1.left_bumper){
                while(gamepad1.left_bumper);
                if(!isfrontOut){
                    frontOut();
                    isfrontOut = true;
                } else {
                    frontIn();
                    isfrontOut = false;
                }
            }

            if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
                startMotors((gamepad1.right_stick_y), (gamepad1.left_stick_y));
            } else {
                stopMotors();
            }

            if(gamepad1.a) {
                while(gamepad1.a);
                reverse();
            }

            if(gamepad2.a) {
                while(gamepad2.a);
                if(liftRelease.getPosition() != 1)
                    activateLift();
                else
                    unactivateLift();
            }

            if(gamepad2.right_bumper)
                grabArms();

            if(gamepad2.left_bumper)
                closeArms();

            if(gamepad2.right_trigger > .05)
                openArms();

            if(gamepad2.left_trigger > .05)
                dropArms();

            if(gamepad2.x) {
                while(gamepad2.x);
                if(armRelease.getPosition() < .5)
                    armRelease();
                else
                    armBlocked();
            }

            if(gamepad2.b) {
                while(gamepad2.b);
                if(topGrabber.getPosition() != .75)
                    topGrab();
                else
                    topUngrab();
            }

            if(gamepad2.y) {
                prepareLift();
            }

        }

        telemetry.addData("shooting mode: ", shootMode + "");
        telemetry.update();

    }
}
