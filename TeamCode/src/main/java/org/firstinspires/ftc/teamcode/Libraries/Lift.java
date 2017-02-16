package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Arib on 1/2/2017.
 */

public class Lift {
    Servo armLeft;
    Servo armRight;
    DcMotor liftRelease;
    Servo armRelease;
    Servo topGrabber;

    LinearOpMode opMode;

    private final double ARM_GRAB = 1;
    private final double ARM_OPEN = .4;
    private final double ARM_DROP = .66;
    private final double ARM_CLOSE = 0;
    private final double LIFT_UNACTIVATED = 0;
    private final double LIFT_ACTIVATED = 1;
    private final double ARM_RELEASER_RELEASED = 1;
    private final double ARM_RELEASER_CLOSED = 0;
    private final double TOP_GRAB = 1;
    private final double TOP_UNGRAB = 0;
    private final double TOP_IDLE = .6;

    public Lift(LinearOpMode opMode) {
        this.opMode = opMode;
        armLeft = opMode.hardwareMap.servo.get("aL");
        armRight = opMode.hardwareMap.servo.get("aR");
        liftRelease = opMode.hardwareMap.dcMotor.get("lrelease");
        armRelease = opMode.hardwareMap.servo.get("arelease");
        topGrabber = opMode.hardwareMap.servo.get("topGrabber");
        grabArms();
        unactivateLift();
        armBlocked();
        topUngrab();
    }

    public void grabArms() {
        armLeft.setPosition(1 - ARM_GRAB);
        armRight.setPosition(ARM_GRAB - .1);
    }

    public void openArms() {
        armLeft.setPosition(ARM_OPEN);
        armRight.setPosition(1 - ARM_OPEN - .05);
    }

    public void dropArms() {
        armLeft.setPosition(ARM_DROP);
        armRight.setPosition(1 - ARM_DROP);
    }

    public void closeArms() {
        armLeft.setPosition(1 - ARM_CLOSE);
        armRight.setPosition(ARM_CLOSE);
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
        liftRelease.setPower(.15);
    }

    public void unactivateLift() {
        liftRelease.setPower(0);
    }

}
