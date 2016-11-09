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

        if(gamepad2.dpad_up) {
            while(gamepad2.dpad_up);
            shooterPower += .05;
        }

        if(gamepad2.dpad_down) {
            while(gamepad2.dpad_down);
            shooterPower -= .05;
        }

        if(gamepad2.left_bumper)
            stopShooter();

        if(gamepad2.a){
            while(gamepad2.a);
            if (!isbackOut) {
                backOut();
                isbackOut = true;
            } else {
                backIn();
                isbackOut = false;
            }
        }

        if(gamepad2.y){
            while(gamepad2.y);
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
//
//        flyRPM = (((shooterL.getCurrentPosition() + shooterR.getCurrentPosition()) / 2) - oldFly) / getRuntime();
//
//        oldFly = ((shooterL.getCurrentPosition() + shooterR.getCurrentPosition()) / 2);
//
//        if(flyTicks > 99) {
//            double sum = 0;
//            for(int i = 0; i < 100; i++) {
//                sum += flyArray[i];
//            }
//            calculatedRPM = (pastCalculatedRPM * .7) + ((sum / 100) * .3);
//            pastCalculatedRPM = calculatedRPM;
//            telemetry.addData("Sum", sum);
//            telemetry.update();
//            flyTicks /= 100;
//        } else {
//            flyArray[flyTicks] = flyRPM;
//            flyTicks++;
//            telemetry.addData("ticks", flyTicks);
//        }
//
//        try {
//            Thread.sleep(10);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        resetStartTime();
//        telemetry.addData("encoderVals", "Right: " + shooterR.getCurrentPosition() + "Left: " + shooterL.getCurrentPosition());
//        telemetry.addData("RPM", calculatedRPM);
//        telemetry.update();
    }
}
