package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Lernaean.Interfaces.DriveInterface;
import org.firstinspires.ftc.teamcode.Lernaean.Interfaces.ServoInterface;

/**
 * Created by Arib on 9/11/2016.
 */
public abstract class LernaeanOpMode extends OpMode implements DriveInterface, ServoInterface {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    Servo rightRight;
    Servo rightLeft;
    Servo leftRight;
    Servo leftLeft;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsMiddle;
    OpticalDistanceSensor odsSide;

    private final double RIGHT_RIGHT_OUT = 1;
    private final double RIGHT_RIGHT_IN = 0;
    private final double RIGHT_LEFT_OUT = 1;
    private final double RIGHT_LEFT_IN = 0;
    private final double LEFT_RIGHT_OUT = 0;
    private final double LEFT_RIGHT_IN = 1;
    private final double LEFT_LEFT_OUT = 0;
    private final double LEFT_LEFT_IN = 1;



    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBR = hardwareMap.dcMotor.get("BR");
        rightRight = hardwareMap.servo.get("RRS");
        rightLeft = hardwareMap.servo.get("RLS");
        leftRight = hardwareMap.servo.get("LRS");
        leftLeft = hardwareMap.servo.get("LLS");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rightColor = hardwareMap.colorSensor.get("rCol");
        leftColor = hardwareMap.colorSensor.get("lCol");
        odsMiddle = hardwareMap.opticalDistanceSensor.get("odsM");
        odsSide = hardwareMap.opticalDistanceSensor.get("odsS");
    }

    public void startMotors(double ri, double le) {
        motorBL.setPower(-le);
        motorBR.setPower(ri);
        motorFL.setPower(-le);
        motorFR.setPower(ri);
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void rightRightOut() {
        rightRight.setPosition(RIGHT_RIGHT_OUT);
    }

    public void rightRightIn() {
        rightRight.setPosition(RIGHT_RIGHT_IN);
    }

    public void rightLeftOut() {
        rightLeft.setPosition(RIGHT_LEFT_OUT);
    }

    public void rightLeftIn() {
        rightLeft.setPosition(RIGHT_LEFT_IN);
    }

    public void leftLeftOut() {
        leftLeft.setPosition(LEFT_LEFT_OUT);
    }

    public void leftLeftIn() {
        leftLeft.setPosition(LEFT_LEFT_IN);
    }

    public void leftRightOut() {
        leftRight.setPosition(LEFT_RIGHT_OUT);
    }

    public void leftRightIn() {
        leftRight.setPosition(LEFT_RIGHT_IN);
    }

}
