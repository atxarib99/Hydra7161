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
public class hydraDrive extends OpMode {
    //creates motors
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    int motorBLE; //These are encoder values for each motor.
    int motorBRE;
    int motorFLE;
    int motorFRE;
    int divider;
    public void resetEncoders() {
        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
    public void startMotors(double power1, double power2, double power3, double power4) {
        motorBR.setPower(power1 * divider);
        motorBL.setPower(power2 * divider);
        motorFL.setPower(power3 * divider);
        motorFR.setPower(power4 * divider);
    }
    public void stopMotors() {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void setDivider(int divide) {
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
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
    }

    //calculates movement updates encoders and looks for buttons pressed
    @Override
    public void loop() {
        motorBRE = motorBR.getCurrentPosition();
        motorBLE = motorBL.getCurrentPosition();
        motorFLE = motorFL.getCurrentPosition();
        motorFRE = motorFR.getCurrentPosition();

        telemetry.addData("motorBL", motorBLE);
        telemetry.addData("motorFR", motorFRE);

        telemetry.addData("motorBR", motorBRE);

        telemetry.addData("motorFL", motorFLE);
        if (Math.abs(gamepad1.left_stick_y) > .5 || Math.abs(gamepad1.right_stick_y) > .5) {
            startMotors(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_y);
        }

        else {
            stopMotors();
        }
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