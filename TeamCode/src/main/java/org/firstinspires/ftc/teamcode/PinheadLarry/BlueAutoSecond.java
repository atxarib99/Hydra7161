package org.firstinspires.ftc.teamcode.PinheadLarry;

/**
 * Created by Arib on 4/19/2016.
 */
public class BlueAutoSecond extends PinheadAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(3, 6, 4);
        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        Thread.sleep(10000);

        moveToCoordinatePos(1,3);

        waitOneFullHardwareCycle();

//        moveToCoordinatePos(3, 5);

//        waitOneFullHardwareCycle();


    }
}
