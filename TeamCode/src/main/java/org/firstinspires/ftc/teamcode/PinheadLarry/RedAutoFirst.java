package org.firstinspires.ftc.teamcode.PinheadLarry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Arib on 4/19/2016.
 */
@Autonomous(name = "Autonomous", group = "LinearOpMode")
public class RedAutoFirst extends PinheadAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(3, 6, 1);
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
