package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;

/**
 * Created by Arib on 1/4/2016.
 */
//Decommissioned class due to old method calls. No longer useful. Outcome is unreliable / inconsistent
public class Autonomous extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {

        first();

        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        moveForward(.5, 8500);

        waitOneFullHardwareCycle();

        if(isOk()) {
            dumpClimbers();

            Thread.sleep(1000);

            resetClimbers();
        }

        waitOneFullHardwareCycle();
    }
}
