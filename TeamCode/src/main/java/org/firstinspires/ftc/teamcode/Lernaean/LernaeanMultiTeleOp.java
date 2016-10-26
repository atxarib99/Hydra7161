package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Arib on 10/20/2016.
 */
@TeleOp(name = "MultiThread", group = "opMode")
public class LernaeanMultiTeleOp extends LernaeanOpMode {
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
            if(driveThread.isAlive())
                driveThread.interrupt();
            driveThread = new Thread(startMotorsR);
            driveThread.start();
        } else {
            if(driveThread.isAlive())
                driveThread.interrupt();
            driveThread = new Thread(stopMotorR);
            driveThread.start();
        }

        if(gamepad1.right_bumper) {
            if(maniThread.isAlive())
                maniThread.interrupt();
            maniThread = new Thread(startManiR);
            maniThread.start();
        }

        if(gamepad1.left_bumper) {
            if(maniThread.isAlive())
                maniThread.interrupt();
            maniThread = new Thread(stopManiR);
            maniThread.start();

        }

        if(gamepad1.a) {
            while(gamepad1.a);
            reverse();
        }

        telemetry.update();
    }
}
