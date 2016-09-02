package org.firstinspires.ftc.teamcode.AutonomousTesting;


import org.firstinspires.ftc.teamcode.AutoMode;

/**
 * Created by Arib on 3/24/2016.
 */
public class CoordinateAuto extends AutoMode {

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
