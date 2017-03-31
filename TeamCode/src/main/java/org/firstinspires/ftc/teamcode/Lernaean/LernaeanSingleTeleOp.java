package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SingleTeleOp", group = "opMode")
public class LernaeanSingleTeleOp extends LernaeanOpMode {
    private boolean shootMode = true;
    private boolean bPressed = false;
    private boolean xPressed = false;

    @Override
    public void loop()
    {
        powerR = gamepad2.right_stick_y;
        powerL = gamepad2.left_stick_y;

        if(shootMode) {

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)) {
                startMotors(powerR, powerL);
            } else if (!bPressed && !xPressed){
                stopMotors();
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
                shooterPower += .05;
                while(gamepad2.dpad_up);
            }

            if(gamepad2.dpad_down) {
                shooterPower -= .05;
                while(gamepad2.dpad_down);
            }

            if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
                reverseMani();
            } else if(gamepad2.right_trigger > .05) {
                if(shooterL.getPower() > 0) {
                    startManiSlow();
                }
                else {
                    startMani();
                    activateShooter(false);
                }
            } else if(gamepad2.left_trigger > .05) {
                stopMani();
            }
        }

        //if we are shooting
        if(!shootMode) {

            if((Math.abs(powerR) > .05 || (Math.abs(powerL) > .05))) {
                startMotorsSlowed((powerR), (powerL));
            }
            //if we are not moving stop the motors
            else if(!xPressed && !bPressed) {
                stopMotors();
            }

            if(gamepad2.y) {
                activateLift();

            } else {
                unactivateLift();
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                armsGrab();
            } else if(gamepad2.right_trigger > .05) {
                armsOut();
            } else if(gamepad2.left_trigger > .05) {
                armsDrop();
            }

            if(gamepad2.x) {
                if(armRelease.getPosition() < .5)
                    armRelease();
                else
                    armBlocked();

                while(gamepad2.x);
            }

            if(gamepad2.a) {
                prepareLift();
            }

        }

        if(gamepad2.dpad_right) {
            reverse();
            while(gamepad2.dpad_right);
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

    }
}
