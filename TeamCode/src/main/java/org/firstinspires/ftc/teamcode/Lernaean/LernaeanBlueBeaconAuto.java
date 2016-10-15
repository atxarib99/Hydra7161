package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by TechMaster on 9/21/2016.
 */
@Autonomous(name="BlueBeaconAuto", group="Linear Opmode")
public class LernaeanBlueBeaconAuto extends LernaeanAutoMode {


    @Override
    public void runOpMode() throws InterruptedException {

        map();

        moveForward(.5, 180); //move forward off wall

        pRotateNoReset(.5, 0); //correct for drift

        pRotate(.5, 45); //turn towards beacons

        while(!leftLine()){
            startMotors(.8, .8); //move forward until middle sensor on line
        }
        stopMotors();

        while(!rightLine()){
            startMotors(-.5, .5); //turn backward until aligned with line
        }
        stopMotors();

        backSemi();

        if (isLeftRed()){           //read color sensor, extend appropriate button pusher
            backIn();
            frontOut();
            Thread.sleep(500);
            frontIn();
        } else {
            backIn();
            Thread.sleep(500);
            backOut();
            Thread.sleep(500);
            backIn();
        }

        moveForward(.5, 200); //move forward off of first line

        while(!rightLine()){
            startMotors(.8, .8); //move forward until middle sensor on line
        }
        stopMotors();

        backSemi();

        if (isLeftRed()){
            backIn();
            frontOut();
            Thread.sleep(500);
            frontIn();
        } else {
            backIn();
            Thread.sleep(500);
            backOut();
            Thread.sleep(500);
            backIn();
        }

}
