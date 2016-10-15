package org.firstinspires.ftc.teamcode.Lernaean;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "opMode")
@Disabled
public class LernaeanTeleOp extends LernaeanOpMode {
    boolean isbackOut = false;
    boolean isfrontOut = false;

    @Override
    public void loop(){

        //booleans YAY!

        if(gamepad1.y){
            if (!isbackOut) {
                backOut();
            } else {
                backIn();
            }
        }

        if(gamepad1.x){
            if(!isfrontOut){
                frontOut();
            } else {
                frontIn();
            }
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_x) > .05)){
            startMotors(gamepad1.right_stick_y,gamepad1.left_stick_y );
        } else {
            stopMotors();
        }

    }
}
