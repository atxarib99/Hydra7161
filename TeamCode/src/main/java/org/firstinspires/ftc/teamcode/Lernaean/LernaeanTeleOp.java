package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    boolean isbackOut = false;
    boolean isfrontOut = false;
    //double flyRPM;
    //double oldFly;
    //double calculatedRPM = 0;
    //double pastCalculatedRPM = 0;

    //double[] flyArray = new double[100];
    //int flyTicks = 0;

    @Override
    public void loop()
    {

        if(gamepad2.right_bumper) {
            startShooter();
        }

        if(gamepad2.x) {
            activateShooter(true);
        }

        if(gamepad2.b) {
            activateShooter(false);
        }

        if(gamepad2.right_stick_y > .05) {
            while(gamepad2.right_stick_y > .05);
            shooterPower += .05;
        }

        if(gamepad2.right_stick_y < -.05) {
            while(gamepad2.right_stick_y < -.05);
            shooterPower -= .05;
        }

        if(gamepad2.left_bumper)
            stopShooter();

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

        if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
            reverseMani();
        } else if(gamepad2.right_trigger > .05) {
            startMani();
        } else if(gamepad2.left_trigger > .05)
            stopMani();

        if(gamepad1.a) {
            while(gamepad1.a);
            reverse();
        }

        if(gamepad2.a) {
            if(liftRelease.getPosition() < .1)
                activateLift();
            else
                unactivateLift();
        }

        if(gamepad2.dpad_up)
            closeArms();

        if(gamepad2.dpad_left)
            openArms();

        if(gamepad2.dpad_down)
            dropArms();

    }
}
