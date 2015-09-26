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
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Autonomous mode
 * enable movement based on sensors and preset code
 */
public class hydraAutonomousBlue extends OpMode {
  //creates motors
  DcMotor motorBL;
  DcMotor motorBR;
  DcMotor motorFL;
  DcMotor motorFR;
  Servo servo1;
  Servo servo2;
//  LightSensor color;
//  OpticalDistanceSensor distance;
    Telemetry test;
    ElapsedTime time = new ElapsedTime();
    double timeCurrent;
    boolean firstLoop;
    boolean secondLoop;

  //defines set motors at the start
  @Override
  public void init() {
    motorBL = hardwareMap.dcMotor.get("motorBL");
    motorBR = hardwareMap.dcMotor.get("motorBR");
    motorFL = hardwareMap.dcMotor.get("motorFL");
    motorFR = hardwareMap.dcMotor.get("motorFR");
    servo1 = hardwareMap.servo.get("servo1");
    servo2 = hardwareMap.servo.get("servo2");
//    color = hardwareMap.lightSensor.get("color"); //THIS MAY BE INCORRECT
//    distance = hardwareMap.opticalDistanceSensor.get("distance"); //THIS MAY BE INCORRECT
      test = new Telemetry();
      timeCurrent = 0.0;
      firstLoop = true;
      secondLoop = false;
  }

  //calculates movement
  @Override
  public void loop() {
      test.addData("Distance", distance.getLightDetected());
      double timeF1 = 1.5;
      double timeF2 = 1.75;
      time.startTime();
      if (firstLoop) {
          while(timeCurrent < timeF1) {
              motorBL.setPower(-1);
              motorFL.setPower(-1);
              motorFR.setPower(1);
              motorBR.setPower(1);
              timeCurrent = time.time();
              if (timeCurrent >= timeF) {
                  firstLoop = false;
                  secondLoop = true;
              }
          }
      }
      if(secondLoop) {
          while (timeCurrent < timeF2) {
              motorBL.setPower(1);
              motorBR.setPower(1);
              motorFL.setPower(1);
              motorFR.setPower(1);
              if(time)
          }

      }




  }

  //stops all motors
  @Override
  public void stop() {
  }
}