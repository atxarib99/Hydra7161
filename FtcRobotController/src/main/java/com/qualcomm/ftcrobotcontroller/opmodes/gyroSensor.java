package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Arib on 10/24/2015.
 */
public class gyroSensor extends LinearOpMode {

    GyroSensor test;
    DeviceInterfaceModule cdim;
    static final int GYRO_CHANNEL = 4;
    public void runOpMode() throws InterruptedException {
        cdim = hardwareMap.deviceInterfaceModule.get("dcim");
        test = hardwareMap.gyroSensor.get("man");
        waitOneFullHardwareCycle();
        waitForStart();
        double rotationValue;
        while (opModeIsActive()) {
            rotationValue = test.getRotation();
            telemetry.addData("rotation", rotationValue);
            waitOneFullHardwareCycle();
        }
    }
}
