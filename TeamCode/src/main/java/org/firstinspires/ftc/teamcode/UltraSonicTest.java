package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by TechMaster on 9/27/2016.
 */
@TeleOp(name = "ustest", group = "OpMode")
public class UltraSonicTest extends OpMode {

    AnalogInput input;
    DeviceInterfaceModule dim;

    @Override
    public void init() {
        telemetry.addData("init", "staring intit you hoe");
        telemetry.update();
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        telemetry.addData("init", "got dim");
        telemetry.update();
        dim.setDigitalChannelMode(0, DigitalChannelController.Mode.OUTPUT);
        telemetry.addData("init", "set output");
        telemetry.update();
        dim.setDigitalChannelState(0, false);
        telemetry.addData("init", "set to false");
        telemetry.update();
        dim.setDigitalChannelMode(1, DigitalChannelController.Mode.INPUT);
        telemetry.addData("init", "finished");
        telemetry.update();
    }

    @Override
    public void loop() {
        try {
            dim.setDigitalChannelState(0, false);
            Thread.sleep(0, 5000);
            dim.setDigitalChannelState(0, true);
            Thread.sleep(0, 10000);
            dim.setDigitalChannelState(0, false);
            telemetry.addData("still waiting", "dawg");
            telemetry.update();
            while(!dim.getDigitalChannelState(1)) {
                wait();
            }
            long startTime = System.nanoTime();
            telemetry.addData("still waiting", "kinda");
            telemetry.update();
            while(dim.getDigitalChannelState(1)) {
                wait();
            }
            long endTime = System.nanoTime();
            telemetry.addData("still waiting", "wait finished");
            telemetry.addData("vals", ((endTime / 1000) - (startTime / 1000)) / 74);
            telemetry.update();
        } catch (Exception e) {
            stop();
        }
    }
}
