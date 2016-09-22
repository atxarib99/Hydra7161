package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Arib on 9/14/2016.
 */
@Autonomous(name="LernaeanRedBeacon", group="Linear Opmode")
public class LerneaenRedBeaconAuto extends LernaeanAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Red Team
        map();

        moveForward(.5, 180); //move forward off wall

        pRotateNoReset(.5, 0); //correct for drift

        pRotate(.5, -45); //turn towards beacons

        while (!middleLine()) {
            startMotors(.8, .8); //move forward until middle sensor on line
        }
        stopMotors();

        while (!sideLine()) {
            startMotors(-.5, .5); //move backward until aligned with line
        }
        stopMotors();

        //Robot should now be in correct position ready to press beacon

        rightRightSemi();       //push button slightly out to detect color

        if (isRightRed()) {      //if the color is red push and we are red team press the red button
            rightRightOut();
            Thread.sleep(500);
            rightRightIn();
        } else {                //else push out the other button
            rightRightIn();
            rightLeftOut();
            Thread.sleep(500);
            rightLeftIn();
        }

        while (!middleLine()) {  //move forward again to the next beacon
            startMotors(.8, .8);
        }

        stopMotors();

        if (isRightRed()) {      //if the color is red push and we are red team press the red button
            rightRightOut();
            Thread.sleep(500);
            rightRightIn();
        } else {                //else push out the other button
            rightRightIn();
            rightLeftOut();
            Thread.sleep(500);
        }
    }
}
