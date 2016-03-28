package com.qualcomm.ftcrobotcontroller.opmodes.AutonomousTesting;

import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;

/**
 * Created by Arib on 3/24/2016.
 */
public class CoordinateAuto extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(5, 2, 4);
        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        moveToCoordinatePos(5, 3);

        waitOneFullHardwareCycle();

//        moveToCoordinatePos(3, 5);

//        waitOneFullHardwareCycle();


    }

}
