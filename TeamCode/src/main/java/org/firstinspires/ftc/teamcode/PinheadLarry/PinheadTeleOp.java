package org.firstinspires.ftc.teamcode.PinheadLarry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Arib on 4/11/2016.
 */


@TeleOp(name = "Tele-OP", group = "OpMode")
@Disabled
public class PinheadTeleOp extends PinheadOpMode {

    boolean rightOpen = false;
    boolean leftOpen = false;
    boolean rightOut = false;
    boolean leftOut = false;
    @Override
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);

        }
        else {
            stopMotors();
        }

        if(gamepad1.a) {
            startMani();
        } else if(gamepad1.x) {
            reverseMani();
        } else if(gamepad1.b) {
            stopMani();
        }

        if(gamepad1.right_bumper) {
            if(rightOut) {
                rightPaddleIn();
                rightOut = false;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                rightPaddleOut();
                rightOut = true;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        if(gamepad1.left_bumper) {
            if(leftOut) {
                leftPaddleIn();
                leftOut = false;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                leftPaddleOut();
                leftOut = true;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        if(gamepad2.right_trigger > .05) {
            extendLifts(gamepad2.right_trigger);
        } else if(gamepad2.left_trigger > .05) {
            extendLifts(-gamepad2.left_trigger);
        } else {
            stopLifts();
        }


        if(gamepad2.x) {
            moveConvLeft();
        }

        if(gamepad2.b) {
            moveConvRight();
        }

        if(gamepad2.a) {
            stopConv();
        }

        if(gamepad2.right_bumper) {
            if(rightOpen) {
                closeRightDoor();
                rightOpen = false;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                openRightDoor();
                rightOpen = true;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        if(gamepad2.left_bumper) {
            if(leftOpen) {
                closeLeftDoor();
                leftOpen = false;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                openLeftDoor();
                leftOpen = true;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        if(gamepad2.right_stick_button) {
            latchDown();
        }

        if(gamepad2.left_stick_button) {
            latchUp();
        }

        getAngles();
    }

}
