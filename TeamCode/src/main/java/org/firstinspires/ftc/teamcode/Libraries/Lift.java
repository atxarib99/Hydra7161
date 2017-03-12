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

    LinearOpMode opMode;

    private final double ARM_GRAB = .68;
    private final double ARM_OPEN = .425;
    private final double ARM_DROP = .66;
    private final double ARM_CLOSE = 0;
    private final double LIFT_UNACTIVATED = 0;
    private final double LIFT_ACTIVATED = 1;
    private final double ARM_IN = .85;
    private final double ARM_RELEASER_RELEASED = 0;
    private final double ARM_RELEASER_CLOSED = 1;
    private final double TOP_GRAB = 1;
    private final double TOP_UNGRAB = 0;
    private final double TOP_IDLE = .6;

    public Lift(LinearOpMode opMode) {
        this.opMode = opMode;
        armLeft = opMode.hardwareMap.servo.get("aL");
        armRight = opMode.hardwareMap.servo.get("aR");
        liftRelease = opMode.hardwareMap.dcMotor.get("lrelease");
        armRelease = opMode.hardwareMap.servo.get("arelease");
<<<<<<< Updated upstream
        topGrabber = opMode.hardwareMap.servo.get("topGrabber");
<<<<<<< Updated upstream
        armsIn();
=======
        grabArms();
=======
        armsIn();
>>>>>>> Stashed changes
>>>>>>> Stashed changes
        unactivateLift();
        armBlocked();
    }

    public void armsIn() {
<<<<<<< Updated upstream
        armLeft.setPosition(.4);
        armRight.setPosition(.6);
=======
<<<<<<< Updated upstream
        armLeft.setPosition(1 - ARM_GRAB - .05);
        armRight.setPosition(ARM_GRAB);
=======
        armLeft.setPosition(.5);
        armRight.setPosition(.5);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
    }

    public void armsOut() {
        armLeft.setPosition(.37);
        armRight.setPosition(.63);
    }

<<<<<<< Updated upstream
    public void armsDrop() {
        armLeft.setPosition(0);
        armRight.setPosition(1);
=======
<<<<<<< Updated upstream
    public void dropArms() {
        armLeft.setPosition(ARM_DROP);
        armRight.setPosition(1 - ARM_DROP);
=======
    public void armsDrop() {
        armLeft.setPosition(.2);
        armRight.setPosition(.8);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
    }

    public void armsGrab() {
        armLeft.setPosition(.53);
        armRight.setPosition(.47);
    }

    public void armRelease() {
        armRelease.setPosition(ARM_RELEASER_RELEASED);
    }

    public void armBlocked() {
        armRelease.setPosition(ARM_RELEASER_CLOSED);
    }

    public void prepareLift() {
        armsDrop();
        armRelease();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        armsOut();
    }

    public void activateLift() {
        liftRelease.setPower(1);
    }

    public void unactivateLift() {
        liftRelease.setPower(0);
    }

}
