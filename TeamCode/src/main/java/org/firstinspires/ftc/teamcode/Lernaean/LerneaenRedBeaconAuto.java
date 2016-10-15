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

        moveForward(-.5, 180); //move backward off wall

        pRotateNoReset(.5, 0); //correct for drift

        pRotate(.5, -45); //turn towards beacons

        while (!leftLine()) {
            startMotors(-.8, -.8); //move forward until middle sensor on line
        }
        stopMotors();

        while (!rightLine()) {
            startMotors(-.5, 0); //turn backward until aligned with line
        }
        stopMotors();

        //Robot should now be in correct position ready to press beacon

        backSemi();       //push button slightly out to detect color

        if (isRightRed()) {      //if the color is red push and we are red team press the red button
            backIn();
            Thread.sleep(500);
            backOut();
            Thread.sleep(500);
            backIn();
        } else {                //else push out the other button
            backIn();
            frontOut();
            Thread.sleep(500);
            frontIn();
        }

        while (!rightLine()) {  //move forward again to the next beacon
            startMotors(-.8, -.8);
        }

        stopMotors();

        backSemi();

        if (isRightRed()) {      //if the color is red push and we are red team press the red button
            backIn();
            frontOut();
            Thread.sleep(500);
            frontIn();
        } else {                //else push out the other button
            backIn();
            frontOut();
            Thread.sleep(500);
            frontIn();
        }
    }
}
