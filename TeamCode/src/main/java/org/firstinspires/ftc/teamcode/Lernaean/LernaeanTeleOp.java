package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    private boolean isbackOut = false;
    private boolean isfrontOut = false;
    private boolean bPressed = false;
    private boolean xPressed = false;

    @Override
    public void loop()
    {
        powerR = gamepad1.right_stick_y;
        powerL = gamepad1.left_stick_y;

        if(shootMode) {

            if(gamepad2.dpad_right) {
                shooterIntegral = 0.0;
                startShooter();
            }

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)) {
                startMotors(powerR, powerL);
            } else if (!bPressed && !xPressed){
                stopMotors();
            }

            if(gamepad1.b) {
                bPressed = true;
                startMotors(-0.8, -0.25);
            }
            else {
                bPressed = false;
            }

            if(gamepad1.x) {
                //while(gamepad1.b);
                xPressed = true;
                startMotors(0.8, 0.25);
            }
            else {
                xPressed = false;
            }

            if(gamepad2.right_bumper) {
                if(voltage < 13.5)
                    shooterIntegral += .007;
                else if(voltage < 13)
                    shooterIntegral += .009;
                else if(voltage < 12.5)
                    shooterIntegral += .01;
                else if(voltage < 12)
                    shooterIntegral += .011;
                else
                    shooterIntegral += .005;
                startShooter();
            }

            if(gamepad2.left_bumper) {
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
                startMani();
                if(shooterL.getPower() > 0) {
                    activateShooter(true);
                }
                else {
                    activateShooter(false);
                }
            } else if(gamepad2.left_trigger > .05)
                stopMani();
        }

        if(!shootMode) {

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)){
                startMotors((powerR * .25), (powerL* .25));
            } else if(!xPressed && !bPressed) {
                stopMotors();
            }

            if(gamepad1.b) {
                bPressed = true;
                startMotors(-0.4, -0.125);
            }
            else {
                bPressed = false;
            }

            if(gamepad1.x) {
                //while(gamepad1.b);
                xPressed = true;
                startMotors(0.4, 0.125);
            }
            else {
                xPressed = false;
            }

            if(gamepad2.y && gamepad1.y) {
                activateLift();

            } else {
                unactivateLift();
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                grabArms();
                topGrab();
            }

            if(gamepad2.right_trigger > .05) {
                openArms();
                topUngrab();
            }

            if(gamepad2.left_trigger > .05) {
                grabArms();
                topUngrab();
            }

            if(gamepad2.x) {
                if(armRelease.getPosition() < .5)
                    armRelease();
                else
                    armBlocked();

                while(gamepad2.x);
            }

            if(gamepad2.b) {
                if(topGrabber.getPosition() != 1)
                    topGrab();
                else
                    topUngrab();

                while(gamepad2.b);
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
                isbackOut = true;
            } else {
                backIn();
                isbackOut = false;
            }
            while(gamepad1.right_bumper);
        }

        if(gamepad1.left_bumper){
            if(front.getPosition() != 0) {
                frontOut();
                isfrontOut = true;
            } else {
                frontIn();
                isfrontOut = false;
            }
            while(gamepad1.left_bumper);
        }

        if (gamepad2.back){
            shootMode = !shootMode;
            while (gamepad2.back);
        }

        currentTime = System.currentTimeMillis();
        currentEncoder = getShooterEncoderAvg();
        velocity = (currentEncoder - lastEncoder) / (currentTime - lastTime);
        velocity *= 1000;
        telemetry.addData("time", currentTime);
        telemetry.addData("Encoder", currentEncoder + "--" + shooterL.getCurrentPosition() + "--" + shooterR.getCurrentPosition());
        if(velocity > 0) {
            velocityAvg[currentTick++] = velocity;
        }
        if(currentTick == 89) {
            lastAvg = avg;
            avg = 0;
            for(int i = 0; i < 90; i++) {
                avg += velocityAvg[i];
            }
            avg /= 90;
            currentTick = 0;
        }
        telemetry.addData("Velocity", avg);
        telemetry.update();
        lastTime = currentTime;
        lastEncoder = currentEncoder;
//        try{
//            Thread.sleep(1);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

    }
}
