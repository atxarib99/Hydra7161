package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Hunter on 9/15/2016.
 */
public abstract class MagazineOpMode extends OpMode {

    DcMotor BL;
    DcMotor BR;
    DcMotor FL;
    DcMotor FR;
    DcMotor Manipulator;
    DcMotor Shooter;
    Servo Magazine;
    Servo rightRight;
    Servo rightLeft;
    Servo leftRight;
    Servo leftLeft;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsMiddle;
    OpticalDistanceSensor odsSide;

    private final double rrIn = 0;
    private final double rrOut = 1;
    private final double rlIn = 0;
    private final double rlOut = 1;
    private final double lrIn = 1;
    private final double lrOut = 0;
    private final double llIn = 1;
    private final double llOut = 0;

    private boolean reverseDrive = false;

    @Override
    public void init(){
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        Manipulator = hardwareMap.dcMotor.get("Manipulator");
        Shooter = hardwareMap.dcMotor.get("Shooter");
        Magazine = hardwareMap.servo.get("Magazine");
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
    public void runMotors(double lf, double rt){
        if (reverseDrive){
            lf *= -1;
            rt *= -1;
        }
        BL.setPower(lf);
        BR.setPower(-rt);
        FL.setPower(lf);
        BR.setPower(-rt);
    }

    public void stopMotors(){
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
    }

    public void runManip(double cl){
        Manipulator.setPower(cl);
    }

    public void runShooter(double sh){
        Shooter.setPower(sh);
    }

    public void runMagazine(double mg){
        Magazine.setPosition(mg);
    }

    public void rightRightOut() {
        rightRight.setPosition(rrOut);
    }

    public void rightRightIn() {
        rightRight.setPosition(rrIn);
    }

    public void rightLeftOut() {
        rightLeft.setPosition(rlOut);
    }

    public void rightLeftIn() {
        rightLeft.setPosition(rlIn);
    }

    public void leftLeftOut() {
        leftLeft.setPosition(llOut);
    }

    public void leftLeftIn() {
        leftLeft.setPosition(llIn);
    }

    public void leftRightOut() { leftRight.setPosition(lrOut); }

    public void leftRightIn() {
        leftRight.setPosition(lrIn);
    }

    public void stopAll(){
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        Shooter.setPower(0);
        Manipulator.setPower(0);

    }

    public void Reverse(){
        if (reverseDrive){
            reverseDrive = false;
        }
        else{
            reverseDrive = true;
        }
    }

    @Override
    public abstract void loop();

    @Override
    public void stop(){

    }
}
