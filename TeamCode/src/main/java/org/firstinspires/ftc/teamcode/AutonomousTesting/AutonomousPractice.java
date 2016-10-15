package org.firstinspires.ftc.teamcode.AutonomousTesting;


import org.firstinspires.ftc.teamcode.OldFiles.AutoMode;

/**
 * Created by Arib on 3/7/2016.
 */
public class AutonomousPractice extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {

        first();

        waitOneFullHardwareCycle();

        waitForStart();

        moveForwardPID(.3, 7500);

        Thread.sleep(500);

        pRotateNoReset(-.3, 0);

        Thread.sleep(500);

        pRotate(.3, 90);

    }
}
