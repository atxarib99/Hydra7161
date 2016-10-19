package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 9/11/2016.
 */
public abstract class LernaeanOpMode extends OpMode {
    DcMotor motorL;
    DcMotor motorR;
    DcMotor manipulator;
    Servo front;
    Servo back;
    DeviceInterfaceModule cdim;
    ColorSensor color;
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;

    Thread driveThread;
    Thread maniThread;
    Thread beaconThread;
    Thread sensorThread;

    private final double BACK_OUT = 1;
    private final double BACK_IN = 0;
    private final double FRONT_OUT = 1;
    private final double FRONT_IN = 0;
    private boolean reversed;



    @Override
    public void init() {
        reversed = false;
        motorL = hardwareMap.dcMotor.get("L");
        motorR = hardwareMap.dcMotor.get("R");
        manipulator = hardwareMap.dcMotor.get("mani");
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        color = hardwareMap.colorSensor.get("color");
        left = hardwareMap.opticalDistanceSensor.get("odsL");
        right = hardwareMap.opticalDistanceSensor.get("odsR");
    }

    public void startMotors(double ri, double le) {
        if(reversed) {
            motorL.setPower(le);
            motorR.setPower(-ri);
        } else {
            motorL.setPower(-le);
            motorR.setPower(ri);
        }
    }

    public void stopMotors() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    public void frontOut() {
        front.setPosition(FRONT_OUT);
    }

    public void frontIn() {
        front.setPosition(FRONT_IN);
    }

    public void backOut() {
        back.setPosition(BACK_OUT);
    }

    public void backIn() {
        back.setPosition(BACK_IN);
    }

    public void startMani() {
        manipulator.setPower(1);
    }

    public void stopMani() {
        manipulator.setPower(0);
    }

    public void reverseMani() {
        manipulator.setPower(-1);
    }

    public void reverse() {
        reversed = !reversed;
    }

}
