package org.firstinspires.ftc.teamcode.Lernaean.DeprecatedFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Arib on 4/19/2016.
 */

@TeleOp(name = "teleOp" , group = "OpMode")
@Disabled
@Deprecated
public class MagazineTeleOp extends MagazineOpMode {
    boolean isFrontOut = false;
    boolean isBackOut = false;

    @Override
    public void loop(){

        telemetry.update();
        if(gamepad2.x){
            if(!isFrontOut){
                frontOut();
                isFrontOut = true;
            } else {
                frontIn();
                isFrontOut = false;
            }
        }

        if(gamepad2.a){
            if(!isBackOut){
                backOut();
                isBackOut = true;
            } else {
                backIn();
                isBackOut = false;
            }
        }

        if(gamepad1.right_bumper) {
            runManip(1);
        }
        if(gamepad1.left_bumper) {
            runManip(0);
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y );
        } else {
            stopMotors();
        }

        if(gamepad1.a){
            reverse();
        }
    }
}
