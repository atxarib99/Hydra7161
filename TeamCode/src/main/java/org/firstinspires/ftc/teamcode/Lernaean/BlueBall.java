package org.firstinspires.ftc.teamcode.Lernaean;

/**
 * Created by Arib on 9/21/2016.
 */
public class BlueBall extends LernaeanAutoMode {

    @Override
    public void runOpMode() throws InterruptedException{
        //Red Team
        map();

        moveForward(.5, 180); //move forward off wall

        pRotateNoReset(.5, 0); //correct for drift

        pRotate(.5, 45); //turn towards beacons

        moveForward(.5, 2000); //knock off the red ball

    }
}