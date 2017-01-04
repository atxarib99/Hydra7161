package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by Arib on 9/11/2016.
 */
public abstract class LernaeanOpMode extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor manipulator;
    DcMotor shooterR;
    DcMotor shooterL;
    Servo front;
    Servo back;
    Servo activate;
    Servo armRight;
    Servo armLeft;
    Servo liftRelease;
    Servo armRelease;
    Servo topGrabber;
    DeviceInterfaceModule cdim;
    ColorSensor color;
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;

    private final double BACK_OUT = 0;
    private final double BACK_IN = 1;
    private final double FRONT_OUT = 0;
    private final double FRONT_IN = 1;
    private final double ARM_GRAB = 1;
    private final double ARM_OPEN = .33;
    private final double ARM_DROP = .66;
    private final double ARM_CLOSE = 0;
    private final double LIFT_UNACTIVATED = 0;
    private final double LIFT_ACTIVATED = 1;
    private final double ARM_RELEASER_RELEASED = 1;
    private final double ARM_RELEASER_CLOSED = 0;
    private final double TOP_GRAB = .75;
    private final double TOP_UNGRAB = 1;

    private boolean reversed;


    public double shooterPower;

    @Override
    public void init() {
        shooterPower = 1;
        reversed = false;
        composeTelemetry();
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator = hardwareMap.dcMotor.get("mani");
        shooterR = hardwareMap.dcMotor.get("sR");
        shooterL = hardwareMap.dcMotor.get("sL");
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        activate = hardwareMap.servo.get("active");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        color = hardwareMap.colorSensor.get("color");
        left = hardwareMap.opticalDistanceSensor.get("odsL");
        right = hardwareMap.opticalDistanceSensor.get("odsR");
        armLeft = hardwareMap.servo.get("aL");
        armRight = hardwareMap.servo.get("aR");
        liftRelease = hardwareMap.servo.get("lrelease");
        armRelease = hardwareMap.servo.get("arelease");
        topGrabber = hardwareMap.servo.get("topGrabber");
        shooterPower = .3;
        frontIn();
        backIn();
        grabArms();
        activateShooter(false);
        unactivateLift();
        armBlocked();
        topGrab();

    }

    public void startMotors(double ri, double le) {
        if(reversed) {
            motorBL.setPower(-le);
            motorFL.setPower(-le);
            motorBR.setPower(ri);
            motorFR.setPower(ri);
        } else {
            motorBL.setPower(ri);
            motorFL.setPower(ri);
            motorBR.setPower(-le);
            motorFR.setPower(-le);
        }
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
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

    public void reverse() {
        reversed = !reversed;
    }

    public void startMani() {
        manipulator.setPower(-1);
    }

    public void stopMani() {
        manipulator.setPower(0);
    }

    public void reverseMani() {
        manipulator.setPower(.5);
    }

    public void startShooter() {
        shooterL.setPower(shooterPower);
        shooterR.setPower(-shooterPower);
    }

    public void reverseShooter() {
        shooterL.setPower(-1);
        shooterR.setPower(1);
    }

    public void stopShooter() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void activateShooter(boolean active) {
        if(active)
            activate.setPosition(1);
        else
            activate.setPosition(0);
    }

    public double getRightODS() {
        return right.getLightDetected();
    }

    public double getLeftODS() {
        return left.getLightDetected();
    }

    public double getRawRightODS() {
        return right.getRawLightDetected();
    }

    public double getRawLeftODS() {
        return left.getRawLightDetected();
    }

    public int getEncoderAvg() {
        return (Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorBL.getCurrentPosition()) +
                Math.abs(motorFR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition())) / 4;
    }

    public int getRed() {
        return color.red();
    }

    public int getBlue() {
        return color.blue();
    }

    private double getShooterPower() {
        return shooterPower;
    }

    public void grabArms() {
        armLeft.setPosition(ARM_GRAB);
        armRight.setPosition(1 - ARM_GRAB);
    }

    public void openArms() {
        armLeft.setPosition(ARM_OPEN);
        armRight.setPosition(1 - ARM_OPEN);
    }

    public void dropArms() {
        armLeft.setPosition(ARM_DROP);
        armRight.setPosition(1 - ARM_DROP);
    }

    public void closeArms() {
        armLeft.setPosition(ARM_CLOSE);
        armRight.setPosition(1 - ARM_CLOSE);
    }

    public void armRelease() {
        armRelease.setPosition(ARM_RELEASER_RELEASED);
    }

    public void armBlocked() {
        armRelease.setPosition(ARM_RELEASER_CLOSED);
    }

    public void topGrab() {
        topGrabber.setPosition(TOP_GRAB);
    }

    public void topUngrab() {
        topGrabber.setPosition(TOP_UNGRAB);
    }

    public void prepareLift() {
        dropArms();
        armRelease();
        topUngrab();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        openArms();
    }

    public void activateLift() {
        liftRelease.setPosition(LIFT_ACTIVATED);
    }

    public void unactivateLift() {
        liftRelease.setPosition(LIFT_UNACTIVATED);
    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("ShooterPower", new Func<String>() {
                    @Override public String value() {
                        return "ShooterPower: " + getShooterPower();
                    }
                });
        telemetry.addLine()
                .addData("BL", new Func<String>() {
                    @Override public String value() {
                        return "BL: " + motorBL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("BR", new Func<String>() {
                    @Override public String value() {
                        return "BR: " + motorBR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FL", new Func<String>() {
                    @Override public String value() {
                        return "FL: " + motorFL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FR", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + motorFR.getCurrentPosition();
                    }
                });
    }



}
