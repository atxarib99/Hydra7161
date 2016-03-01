package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;

/**
 * Created by Arib on 1/4/2016.
 */
public class Autonomous2 extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {

        first();

        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        moveForward(.5, 8500);

        waitOneFullHardwareCycle();

        if(allIsOk()) {
            rotate();
        }

        waitOneFullHardwareCycle();

        if(allIsOk()) {
            dumpClimbers();
        }

        waitOneFullHardwareCycle();
    }
}