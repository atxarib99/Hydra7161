package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lernaean.MagazineOpMode;

/**
 * Created by Arib on 4/19/2016.
 */

public class MagazineTeleOp extends MagazineOpMode {
    boolean isrightRightOut = false;
    boolean isrightLeftOut = false;
    boolean isleftRightOut = false;
    boolean isleftLeftOut = false;

    @Override
    public void loop(){
        if(gamepad2.y){
            if (!isrightRightOut) {
                rightRightOut();
                isrightRightOut = true;
            } else {
                rightRightIn();
                isrightRightOut = false;
            }
        }

        if(gamepad2.x){
            if(!isrightLeftOut){
                rightLeftOut();
                isrightLeftOut = true;
            } else {
                rightRightIn();
                isrightLeftOut = false;
            }
        }

        if (gamepad2.b){
            if(!isleftRightOut){
                leftRightOut();
                isleftRightOut = true;
            } else {
                leftRightIn();
                isleftRightOut = false;
            }
        }

        if(gamepad2.a){
            if(!isleftLeftOut){
                leftLeftOut();
                isleftLeftOut = true;
            } else {
                leftRightIn();
                isleftLeftOut = false;
            }
        }

        if(gamepad2.left_trigger > .05){

        }

        if(gamepad1.right_bumper) {
            runManip(1);
        }
        if(gamepad1.left_bumper) {
            runManip(0);
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_x) > .05)){
            runMotors(gamepad1.right_stick_y,gamepad1.left_stick_y );
        } else {
            stopMotors();
        }
    }
    @Override
    public void stop(){

    }
}
