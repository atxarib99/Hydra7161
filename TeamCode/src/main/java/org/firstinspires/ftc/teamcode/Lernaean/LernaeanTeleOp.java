package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LTeleOp", group = "opMode")
public class LernaeanTeleOp extends LernaeanOpMode {
    private boolean isbackOut = false;
    private boolean isfrontOut = false;
    private boolean shootMode = true;
    private boolean movingForward = true;
    private double currentTime = 0;
    private double currentEncoder = 0;
    private double lastTime = 0;
    private double lastEncoder = 0;
    private double velocity = 0.0;

    @Override
    public void loop()
    {
        powerR = gamepad1.right_stick_y;
        powerL = gamepad1.left_stick_y;
//        currentTime = time.milliseconds();
//        currentEncoder = getEncoderAvg();
//        velocity = (currentEncoder - lastEncoder) / (currentTime - lastTime);
//        telemetry.addData("Velocity", velocity);
//        telemetry.update();
//        lastTime = currentTime;
//        lastEncoder = currentEncoder;
        currentTime = System.currentTimeMillis();
        currentEncoder = getShooterEncoderAvg();
        velocity = (currentEncoder - lastEncoder) / (currentTime - lastTime);
        telemetry.addData("time", currentTime);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Encoder", currentEncoder);
        telemetry.addData("shooting mode: ", shootMode + "");
        telemetry.update();
        lastTime = currentTime;
        lastEncoder = currentEncoder;

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

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)) {
//                if(powerR > .05 || powerL > .05) {
//                    if(motorBL.getPower() < .05 || motorBR.getPower() < .05) {
//                        try {
//                            pulseStop();
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }
//                    }
//                    else {
//                        startMotors(powerR, powerL);
//                    }
//                }
//                if(powerR < .05 || powerL < .05) {
//                    if(motorBL.getPower() > .05 || motorBL.getPower() > .05) {
//                        try {
//                            pulseStop();
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }
//                    }
//                    else {
//                        startMotors(powerR, powerL);
//                    }
//                }
                startMotors(powerR, powerL);
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

            if (Math.abs(powerR) > .05 || (Math.abs(powerL) > .05)){
                startMotors((powerR), (powerL));
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
    }
}
