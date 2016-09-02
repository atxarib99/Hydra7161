package org.firstinspires.ftc.teamcode.PinheadLarry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Arib on 4/19/2016.
 */
@Autonomous(name = "Autonomous2", group = "LinearOpMode")
@Disabled
public class RedAutoSecond extends PinheadAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        cordFirst(3, 6, 1);
        waitOneFullHardwareCycle();

        waitForStart();

        waitOneFullHardwareCycle();

        Thread.sleep(10000);

        moveToCoordinatePos(3,1);

        waitOneFullHardwareCycle();

//        moveToCoordinatePos(3, 5);

//        waitOneFullHardwareCycle();


    }
}
