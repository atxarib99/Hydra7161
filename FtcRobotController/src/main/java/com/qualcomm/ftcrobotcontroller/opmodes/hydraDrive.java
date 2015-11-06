/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class hydraDrive extends OpMode implements hydraDriveBase, LiftInterface, ClawInterface {
    //creates motors
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
//    DcMotor lift;
    DcMotor rightClaw;
    DcMotor leftClaw;
//    DcMotor manipulator;
//    Servo basket;

    double motorBLE; //These are encoder values for each motor.
    double motorBRE;
    double motorFLE;
    double motorFRE;
    int divider;
    public void resetEncoders() {
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
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
        rightClaw.setPower(change);
        leftClaw.setPower(-change);
    }
    public void stopClaw() {
        rightClaw.setPower(0);
        leftClaw.setPower(0);

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
        motorBRE = 0;
        motorFLE = 0;
        motorBLE = 0;
        motorFRE = 0;
        divider = 1;
//        lift = hardwareMap.dcMotor.get("lift");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
  //      basket = hardwareMap.servo.get("basket");
        rightClaw = hardwareMap.dcMotor.get("rightClaw");
        leftClaw = hardwareMap.dcMotor.get("leftClaw");

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
        if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
            startMotors(gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y, gamepad1.right_stick_y);
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
            changeClaw(gamepad2.right_trigger);
        }
        else {
            stopClaw();
        }

        if(gamepad2.left_trigger > .05) {
            changeClaw(-gamepad2.left_trigger);
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
            resetEncoders();
        }
        //sets speed to 1/2
        if (gamepad1.x) {
            setDivider(2);
        }
        //sets speed to 1/4
        if (gamepad1.b) {
            setDivider(4);
        }
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