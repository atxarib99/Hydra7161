package com.qualcomm.ftcrobotcontroller.opmodes.AutonomousTesting;

import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;

/**
 * Created by Arib on 3/7/2016.
 */
public class AutonomousPractice extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        first();

        waitForStart();

        moveForward(.3, 500);

        rotate();

    }
}
