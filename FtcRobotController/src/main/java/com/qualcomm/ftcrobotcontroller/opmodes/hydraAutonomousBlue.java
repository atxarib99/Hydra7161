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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Autonomous mode
 * enable movement based on sensors and preset code
 */
public class hydraAutonomousBlue extends LinearOpMode {
    //creates motors
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
//    ColorSensor color;
    //  OpticalDistanceSensor distance
    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
//        color = hardwareMap.colorSensor.get("color");
        int distance1 = 5500;
        int distance2 = 7100;
        int distance3 = 12700;
        while (motorBL.getCurrentPosition() < distance1) {
            motorBL.setPower(1);
            motorBR.setPower(-1);
            motorFL.setPower(1);
            motorFR.setPower(-1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        while (motorBL.getCurrentPosition() < distance2) {
            motorBL.setPower(1);
            motorFL.setPower(1);
            motorFR.setPower(1);
            motorBR.setPower(1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        while (motorBL.getCurrentPosition() < distance3) {
            motorBL.setPower(1);
            motorBR.setPower(-1);
            motorFL.setPower(1);
            motorFR.setPower(-1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        time.startTime();
        double currentTime = 0.0;
        double finalTime = 5.0;
        while (currentTime < finalTime) {

            telemetry.addData("currentTime", time.time());
            currentTime = time.time();
        }
        while (motorBL.getCurrentPosition() > 9000) { //9034
            motorBL.setPower(-1);
            motorBR.setPower(1);
            motorFL.setPower(-1);
            motorFR.setPower(1);
            motorFR.setPower(-1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        while (motorBL.getCurrentPosition() > 9800) {
            motorBL.setPower(1);
            motorBR.setPower(1);
            motorFR.setPower(1);
            motorFL.setPower(1);
            motorFR.setPower(-1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        time.reset();
        currentTime = 0;
        while (currentTime < 5.0) {
            motorBL.setPower(1);
            motorBR.setPower(-1);
            motorFL.setPower(1);
            motorFR.setPower(-1);
            motorFR.setPower(-1);
            telemetry.addData("motorBL", motorBL.getCurrentPosition());

            telemetry.addData("motorFR", motorFR.getCurrentPosition());

            telemetry.addData("motorBR", motorBR.getCurrentPosition());

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
        }
        motorBL.close();
        motorFL.close();
        motorBR.close();
        motorFR.close();
    }
}