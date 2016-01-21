package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Arib on 1/21/2016.
 */
public class touchSensorTest extends OpMode {
    MediaPlayer song;
    DeviceInterfaceModule cdim;
    TouchSensor rts;
    TouchSensor lts;
    @Override
    public void init() {
        song = MediaPlayer.create(FtcRobotControllerActivity.getContext(), R.raw.fsong);
        song.setLooping(true);
        song.start();
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rts = hardwareMap.touchSensor.get("rts");
        lts = hardwareMap.touchSensor.get("lts");

    }


    public void loop() {
        telemetry.addData("rts", rts.isPressed());
        telemetry.addData("lts", lts.isPressed());
        telemetry.addData("rtsToString", rts.toString());
        telemetry.addData("ltsToString", lts.toString());

    }
}
