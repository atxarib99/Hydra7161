package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    private boolean shootMode = true;
    private boolean bPressed = false;
    private boolean xPressed = false;

    @Override
    public void loop() {
        powerR = gamepad1.right_stick_y;
        powerL = gamepad1.left_stick_y;

        divisor = 1 - ((gamepad1.right_trigger * .5) + (gamepad1.left_trigger * .25));

        if(shootMode) {

            if(gamepad2.right_stick_button) {
                startShooter();
                topGrab();
            }

            if(gamepad2.left_stick_button) {
                topUngrab();
                stopShooter();
            }

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)) {
                startMotors(powerR, powerL);
            } else if (!bPressed && !xPressed){
                stopMotors();
            }

            if(gamepad1.b) {
                bPressed = true;
                if(reversed)
                    startMotors(-0.8, -0.25);
                else
                    startMotors(-.25, -.8);
            }
            else {
                bPressed = false;
            }

            if(gamepad1.x) {
                //while(gamepad1.b);
                xPressed = true;
                if(reversed)
                    startMotors(0.8, 0.25);
                else
                    startMotors(.25, .8);
            }
            else {
                xPressed = false;
            }

            if(gamepad2.right_bumper) {
                stopCommandGiven = false;
                activateShooter(true);
                startShooter();
            }

            if(gamepad2.left_bumper) {
                stopCommandGiven = true;
                stopShooter();
            }

            if(gamepad2.x) {
                activateShooter(true);
            }

            if(gamepad2.b) {
                activateShooter(false);
            }

            if(gamepad2.dpad_up) {
                rpm += .05;
                while(gamepad2.dpad_up);
            }

            if(gamepad2.dpad_down) {
                rpm -= .05;
                while(gamepad2.dpad_down);
            }

            if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
                reverseMani();
            } else if(gamepad2.right_trigger > .05) {
                if(Math.abs(shooterL.getPower()) > 0) {
                    activateShooter(true);
                    startManiSlow();
                }
                else {
                    activateShooter(false);
                    startMani();
                }
            } else if(gamepad2.left_trigger > .05) {
                stopMani();
            }
        }

        //if we are not shooting
        if(!shootMode) {

            if((Math.abs(powerR) > .05 || (Math.abs(powerL) > .05))) {
                startMotorsSlowed((powerR), (powerL));
            }
            //if we are not moving stop the motors
            else if(!xPressed && !bPressed) {
                stopMotors();
            }

            //glide against the walls
            if(gamepad1.b) {
                bPressed = true;
                if(reversed)
                    startMotors(-0.4, -0.125);
                else
                    startMotors(-.125, -.4);
            }
            else {
                bPressed = false;
            }

            if(gamepad1.x) {
                //while(gamepad1.b);
                xPressed = true;
                if(reversed)
                    startMotors(0.4, 0.125);
                else
                    startMotors(.125, .4);
            }
            else {
                xPressed = false;
            }

            if(gamepad2.y && gamepad1.y) {
                activateLift();
                liftIsActive = true;
            } else {
                unactivateLift();
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                armsGrab();
                topGrab();
            } else if(gamepad2.right_trigger > .05) {
                armsOut();
                topUngrab();
            } else if(gamepad2.left_trigger > .05) {
                armsDrop();
                topUngrab();
            }

            if(gamepad2.x) {
                if(armRelease.getPosition() > .5)
                    armRelease();
                else
                    armBlocked();

                while(gamepad2.x);
            }

            if(gamepad2.a) {
                prepareLift();
            }

        }

        if(gamepad1.a) {
            reverse();
            while(gamepad1.a);
        }

        //if all of our motors are not running get the voltage
        if(motorBL.getPower() == 0 && motorBR.getPower() == 0 && motorFL.getPower() == 0 &&
                motorFR.getPower() == 0 && manipulator.getPower() == 0 && shooterR.getPower() == 0
                && shooterL.getPower() == 0 && liftRelease.getPower() == 0) {
            voltage = getVoltage();
        }

        if(gamepad1.right_bumper){
            if (back.getPosition() != 0) {
                backOut();
            } else {
                backIn();
            }
            while(gamepad1.right_bumper);
        }

        if(gamepad1.left_bumper){
            if(front.getPosition() != 0) {
                frontOut();
            } else {
                frontIn();
            }
            while(gamepad1.left_bumper);
        }

        if (gamepad2.back){
            shootMode = !shootMode;
            while (gamepad2.back);
        }

        telemetry.update();
    }
}
