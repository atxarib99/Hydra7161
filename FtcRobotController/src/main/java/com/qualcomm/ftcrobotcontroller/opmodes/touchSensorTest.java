package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Arib on 1/21/2016.
 */
public class touchSensorTest extends OpMode {
    MediaPlayer song;
    DeviceInterfaceModule cdim;

    DigitalChannel rts;
    DigitalChannel lts;
    @Override
    public void init() {
        song = MediaPlayer.create(FtcRobotControllerActivity.getContext(), R.raw.fsong);
        song.setLooping(true);
        song.start();
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        rts = hardwareMap.digitalChannel.get("rts");
        lts = hardwareMap.digitalChannel.get("rts");

    }


    public void loop() {
        telemetry.addData("rts", rts.getState());
        telemetry.addData("lts", lts.getState());
        boolean digValR = rts.getState();
        boolean digValL = lts.getState();
        telemetry.addData("rtsToString", String.format("%1d", (digValR ? 1 : 0)));
        telemetry.addData("ltsToString", String.format("%1d", (digValL ? 1 : 0)));

    }
}
