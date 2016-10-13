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
    Servo front;
    Servo back;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsMiddle;
    OpticalDistanceSensor odsSide;

    private final double BACK_OUT = 1;
    private final double BACK_IN = 0;
    private final double FRONT_OUT = 1;
    private final double FRONT_IN = 0;



    @Override
    public void init() {
        motorL = hardwareMap.dcMotor.get("L");
        motorR = hardwareMap.dcMotor.get("R");
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rightColor = hardwareMap.colorSensor.get("rCol");
        leftColor = hardwareMap.colorSensor.get("lCol");
        odsMiddle = hardwareMap.opticalDistanceSensor.get("odsM");
        odsSide = hardwareMap.opticalDistanceSensor.get("odsS");
    }

    public void startMotors(double ri, double le) {
        motorL.setPower(-le);
        motorR.setPower(ri);
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

}
