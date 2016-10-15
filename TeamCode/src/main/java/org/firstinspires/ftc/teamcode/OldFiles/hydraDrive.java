
package org.firstinspires.ftc.teamcode.OldFiles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class hydraDrive extends OpMode {
    //creates motors
    DcMotor motorBL; //motors for movement
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
//    DcMotor lift;
    DcMotor rightClaw; //motors to control claws
    DcMotor leftClaw;
//    DcMotor manipulator;
//    Servo basket;
    Servo rightBar; //servos to control side bars
    Servo leftBar;
    Servo climberBar; //servo to control climber dumping
    boolean frontMotors;

    int divider;
    @Deprecated
    public void resetEncoders() {
        //method is empty because deprecated.
    }
    public void startMotors(double power1, double power2, double power3, double power4) {
        motorBR.setPower(power1 / divider);
        motorBL.setPower(power2 / divider);
        motorFL.setPower(power3 / divider);
        motorFR.setPower(power4 / divider);
    }
    public int getEncoderAvg() {
        return (Math.abs(motorBL.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition()) +
                Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorFR.getCurrentPosition())) / 4;
    }
    public void stopMotors() {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void changeLift(double change) {
//        lift.setPower(change);
    }

    public void stopLift() {
//        lift.setPower(0.0);
    }

    public void changeClaw(double change) {
        rightClaw.setPower(-change);
        leftClaw.setPower(change);
    }
    public void stopClaw() {
        rightClaw.setPower(0);
        leftClaw.setPower(0);

    }
    public void extendBars() {
        rightBar.setPosition(1);
        leftBar.setPosition(0);
    }
    public void contractBars() {
        rightBar.setPosition(0);
        leftBar.setPosition(1);
    }
    public void dumpClimbers() {
        climberBar.setPosition(Servo.MIN_POSITION);
    }
    public void contractClimbers() {
        climberBar.setPosition(Servo.MAX_POSITION);
    }
/*    private void decreaseSpeed() {
        boolean rightTrue = false;
        boolean leftTrue = false;
        if(motorBR.getPower() > .2) { rightTrue = true; }
        if(motorBL.getPower() > .2) { leftTrue = true; }
        while(rightTrue && leftTrue) {
            motorBR.setPower(motorBR.getPower() / 4);
            motorFL.setPower(motorFL.getPower() / 4);
            motorFR.setPower(motorFR.getPower() / 4);
            motorBL.setPower(motorBL.getPower() / 4);
            if(motorBR.getPower() < .2) {
                rightTrue = false;
            }
            if(motorBL.getPower() < .2) {
                leftTrue = false;
            }
        }
        while(rightTrue && !leftTrue){
            motorBR.setPower(motorBR.getPower() / 4);
            motorFR.setPower(motorFR.getPower() / 4);
            if(motorBR.getPower() < .2) {
                rightTrue = false;
            }

        }
        while(!rightTrue && leftTrue){
            motorBL.setPower(motorBL.getPower() / 4);
            motorFL.setPower(motorFL.getPower() / 4);
            if(motorBL.getPower() < .2) {
                leftTrue = false;
            }

        }

    } */
//    private void startManipulater() {
//        manipulater.setPower(1);
//    }
//    private void stopManipulater() {
//        manipulater.setPower(0);
//    }
//    private void reverseManipulater() {
//        manipulater.setPower(-1);
//    }
    private void setDivider(int divide) {
        divider = divide;
    }
    //defines set motors at the start
    @Override
    public void init() {
        divider = 1;
//        lift = hardwareMap.dcMotor.get("lift");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
  //      basket = hardwareMap.servo.get("basket");
        rightClaw = hardwareMap.dcMotor.get("rightClaw");
        leftClaw = hardwareMap.dcMotor.get("leftClaw");
        rightBar = hardwareMap.servo.get("rightBar");
        leftBar = hardwareMap.servo.get("leftBar");
        climberBar = hardwareMap.servo.get("climberBar");
        climberBar.setPosition(Servo.MAX_POSITION);
        rightBar.setPosition(0);
        leftBar.setPosition(1);
        frontMotors = false;

    }

    //calculates movement updates encoders and looks for buttons pressed
    @Override
    public void loop() {
        telemetry.addData("EncoderAverage", getEncoderAvg());
        telemetry.addData("motorBL", motorBR.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("motorBR", motorBR.getCurrentPosition());

        telemetry.addData("motorFL", motorFL.getCurrentPosition());

        //controls motion motors
        if ((Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) && frontMotors) {
            startMotors(gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
        else if((Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) && !frontMotors) {
            startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y, 0, 0);
        }
        else {
            stopMotors();
        }

        //raises lift
        if(Math.abs(gamepad2.right_stick_y) > .05)
            changeLift(-gamepad2.right_stick_y);
        else
            stopLift();

//        //moves basket
//        if(gamepad2.left_stick_x < -.05)
//            if(basket.getPosition() == Servo.MAX_POSITION)
//                basket.setPosition(Servo.MIN_POSITION);
//            basket.setPosition(basket.getPosition() + 1);
//
//        if(gamepad2.left_stick_x > .05)
//            if(basket.getPosition() <= Servo.MIN_POSITION)
//                basket.setPosition(Servo.MAX_POSITION);
//            basket.setPosition(basket.getPosition() - 1);

        //manipulates hooks/claws for hang.
        if(gamepad2.right_trigger > .05) {
            rightClaw.setPower(-gamepad2.right_trigger);
            leftClaw.setPower(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > .05) {
            rightClaw.setPower(gamepad2.left_trigger * .75);
            leftClaw.setPower(-gamepad2.left_trigger * .75);
        }
        else {
            stopClaw();
        }
//        if (gamepad2.right_bumper) {
//            startManipulater();
//        }
//        else
//            stopManipulater();
//        if (gamepad2.left_bumper) {
//            reverseManipulater();
//        }
//        else
//            stopManipulater();
        //resets encoders
        if (gamepad1.a) {
            frontMotors = false;
        }
        //sets speed to 1/2
        if (gamepad1.x) {
            setDivider(2);
        }
        //sets speed to 1/4
        if (gamepad1.b) {
            frontMotors = true;
        }
        //extends side bars to release climbers
        if(gamepad2.right_bumper)
            extendBars();
        //contracts side bars to pull back bars
        if(gamepad2.left_bumper)
            contractBars();
        //rotates bar to dump
        if(gamepad2.a)
            dumpClimbers();
        //pulls climber bar back
        if(gamepad2.b)
            contractClimbers();
        //resets speed to normal
        if(gamepad1.y) {
            setDivider(1);
        }
    }

    //stops all motors
    @Override
    public void stop() {
        resetEncoders();
    }
}