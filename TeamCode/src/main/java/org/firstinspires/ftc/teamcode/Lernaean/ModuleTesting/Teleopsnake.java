package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 11/8/2016.
 */

@TeleOp(name = "snakebytetele", group = "teleop")
@Disabled
public class Teleopsnake extends OpMode {

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFR;
    DcMotor motorFL;

    DcMotor motorDoor;
    DcMotor mani;

    DcMotor shooterF;
    DcMotor shooterB;

    Servo beacon;

    double shooterPower = .5;
    @Override
    public void init() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");

        motorDoor = hardwareMap.dcMotor.get("ManLift");
        mani = hardwareMap.dcMotor.get("ManIn");

        shooterB = hardwareMap.dcMotor.get("B");
        shooterF = hardwareMap.dcMotor.get("F");

        beacon = hardwareMap.servo.get("beacon");
    }

    @Override
    public void loop() {
        if(gamepad2.right_bumper) {
            shooterB.setPower(-shooterPower);
            shooterF.setPower(shooterPower);
        }

        if(gamepad2.dpad_up) {
            while(gamepad2.dpad_up);
            shooterPower += .05;
        }

        if(gamepad2.dpad_down) {
            while(gamepad2.dpad_down);
            shooterPower -= .05;
        }

        if(gamepad2.left_bumper) {
            shooterB.setPower(0);
            shooterF.setPower(0);
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
            motorFL.setPower(gamepad1.left_stick_y);
            motorFR.setPower(-gamepad1.right_stick_y);
            motorBR.setPower(-gamepad1.right_stick_y);
            motorBL.setPower(gamepad1.left_stick_y);
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }

        if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
            mani.setPower(.75);
        } else if(gamepad2.right_trigger > .05) {
            mani.setPower(-.75);
        } else if(gamepad2.left_trigger > .05)
            mani.setPower(0);

        if(gamepad2.right_stick_y < -.05) {
            motorDoor.setPower(.5);
        }
        else if(gamepad2.right_stick_y > .05) {
            motorDoor.setPower(-.2);
        } else {
            motorDoor.setPower(0);
        }

        if(gamepad1.a)
            beacon.setPosition(1);
        if(gamepad1.b)
            beacon.setPosition(0);
        if(gamepad1.x)
            beacon.setPosition(.5);

    }

    @Override
    public void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorDoor.setPower(0);
        shooterB.setPower(0);
        shooterF.setPower(0);
    }
}