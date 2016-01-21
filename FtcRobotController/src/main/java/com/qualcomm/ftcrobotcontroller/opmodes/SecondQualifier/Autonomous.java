package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;

/**
 * Created by Arib on 1/4/2016.
 */
public class Autonomous extends AutoMode {
    @Override
    public void runOpMode() {
        try {
            waitOneFullHardwareCycle();
        } catch(InterruptedException e) {
            stop();
        }
        try {
            waitForStart();
        } catch (InterruptedException e) {
            stop();
        }
        first();
        moveForward(.5);

        raiseLifts(.5, 1);
        myWait(500);
        dumpClimbers();
        myWait(500);

        resetClimbers();
        myWait(500);

        dumpClimbers();
        myWait(500);

        resetClimbers();
        myWait(500);

    }
}
