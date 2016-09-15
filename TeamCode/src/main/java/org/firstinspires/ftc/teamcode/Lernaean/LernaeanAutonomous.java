package org.firstinspires.ftc.teamcode.Lernaean;

/**
 * Created by Arib on 9/14/2016.
 */
public class LernaeanAutonomous extends LernaeanAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {

        map();

        moveForward(.5, 180); //move forward off wall

        pRotateNoReset(.5, 0); //correct for drift

        pRotate(.5, -45); //turn towards beacons

        while(!middleLine()){
            startMotors(.8, .8); //move forward until middle sensor on line
        }
        stopMotors();

        while(!sideLine()){
            startMotors(-.5, .5); //move backward until aligned with line
        }
        stopMotors();
    }
}
