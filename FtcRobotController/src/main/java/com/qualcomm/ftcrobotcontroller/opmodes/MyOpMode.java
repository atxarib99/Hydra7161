//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import java.util.concurrent.TimeUnit;

public abstract class MyOpMode {
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public Telemetry telemetry = new Telemetry();
    public HardwareMap hardwareMap = new HardwareMap();
    public double time = 0.0D;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor liftL;
    DcMotor liftR;
    Servo climberBar;


    private long a = 0L;


    public MyOpMode() {
        this.a = System.nanoTime();
    }

    public abstract void init();

    public void init_loop() {
    }

    public void start() {
    }

    public abstract void loop();

    public void stop() {
    }

    public double getRuntime() {
        double var1 = (double)TimeUnit.SECONDS.toNanos(1L);
        return (double)(System.nanoTime() - this.a) / var1;
    }

    public void resetStartTime() {
        this.a = System.nanoTime();
    }
}
