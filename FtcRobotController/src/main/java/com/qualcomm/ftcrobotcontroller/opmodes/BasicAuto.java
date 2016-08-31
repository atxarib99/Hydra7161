package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 8/29/2016.
 */
public class BasicAuto extends LinearOpMode {
    DcMotor motor1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("banana");
        ElapsedTime counter = new ElapsedTime();
        counter.startTime();
        while(counter.time() < 5.0) {
            motor1.setPower(1);
        }
        motor1.setPower(0);
        int nullEncoder = motor1.getCurrentPosition();
        while(motor1.getCurrentPosition() - nullEncoder < 1440 && time < 1.0) {
            motor1.setPower(1);
        }
        motor1.setPower(0);



    }
}
