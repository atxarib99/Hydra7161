package com.qualcomm.ftcrobotcontroller.opmodes.PinheadLarry;

/**
 * Created by Arib on 4/19/2016.
 */
public class PinheadGrid extends PinheadAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(2, 6, 1);
        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        moveToCoordinatePos(6, 3);

        waitOneFullHardwareCycle();

//        moveToCoordinatePos(3, 5);

//        waitOneFullHardwareCycle();


    }
}