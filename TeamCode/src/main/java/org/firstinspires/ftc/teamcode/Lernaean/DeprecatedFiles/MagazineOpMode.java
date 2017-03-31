package org.firstinspires.ftc.teamcode.Lernaean.DeprecatedFiles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by Hunter on 9/15/2016.
 */
@Deprecated
public abstract class MagazineOpMode extends OpMode {

    DcMotor motorR;
    DcMotor motorL;
    DcMotor manipulator;
    DcMotor shooter;
    Servo front;
    Servo back;
    DeviceInterfaceModule cdim;
    ColorSensor rightColor;
    ColorSensor leftColor;
    OpticalDistanceSensor odsRight;
    OpticalDistanceSensor odsLeft;

    private final double backIn = 0;
    private final double backOut = 1;
    private final double frontIn = 0;
    private final double frontOut = 1;

    private boolean reverseDrive = false;

    @Override
    public void init(){
        composeTelemetry();
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        manipulator = hardwareMap.dcMotor.get("mani");
        //shooter = hardwareMap.dcMotor.get("shooter");
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //rightColor = hardwareMap.colorSensor.get("rCol");
        //leftColor = hardwareMap.colorSensor.get("lCol");
        odsRight = hardwareMap.opticalDistanceSensor.get("odsR");
        odsLeft = hardwareMap.opticalDistanceSensor.get("odsL");
        telemetry.addData("init", "finsihed");
        telemetry.update();

    }
    public void startMotors(double lf, double rt){
        if (reverseDrive){
            lf *= -1;
            rt *= -1;
        }
        motorR.setPower(-rt);
        motorL.setPower(lf);
    }

    public void stopMotors(){
        motorL.setPower(0);
        motorR.setPower(0);
    }

    public void runManip(double cl){
        manipulator.setPower(cl);
    }

    public void runShooter(double sh){
        shooter.setPower(sh);
    }

    public void backOut() {
        back.setPosition(backOut);
    }

    public void backIn() {
        back.setPosition(backIn);
    }

    public void frontOut() {
        front.setPosition(frontOut);
    }

    public void frontIn() {
        front.setPosition(frontIn);
    }


    public void stopAll(){
        motorL.setPower(0);
        motorR.setPower(0);
        shooter.setPower(0);
        manipulator.setPower(0);

    }

    public double getRightODS() {
        return odsRight.getLightDetected();
    }

    public double getLeftODS() {
        return odsLeft.getLightDetected();
    }

    public double getRawRightODS() {
        return odsRight.getRawLightDetected();
    }

    public double getRawLeftODS() {
        return odsLeft.getRawLightDetected();
    }

    public int getEncoderAvg() {
        return (Math.abs(motorR.getCurrentPosition()) + Math.abs(motorL.getCurrentPosition())) / 2;
    }

    public void reverse(){
        if (reverseDrive){
            reverseDrive = false;
        }
        else{
            reverseDrive = true;
        }
    }

    public void composeTelemetry() {

        telemetry.addLine()
                .addData("rightODS", new Func<String>() {
                    @Override public String value() {
                        return "Right: Raw: " + getRawRightODS() + " Normal: " + getRightODS() + "";
                    }
                });

        telemetry.addLine()
                .addData("leftODS", new Func<String>() {
                    @Override public String value() {
                        return "Left: Raw: " + getRawLeftODS() + " Normal: " + getLeftODS() + "";
                    }
                });

        telemetry.addLine()
                .addData("encoder", new Func<String>() {
                    @Override public String value() {
                        return getEncoderAvg() + "";
                    }
                });

    }

    @Override
    public void stop(){
        motorL.setPower(0);
        motorR.setPower(0);
    }
}
