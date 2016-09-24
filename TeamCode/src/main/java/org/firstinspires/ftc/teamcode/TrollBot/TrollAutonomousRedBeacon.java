package org.firstinspires.ftc.teamcode.TrollBot;

/**
 * Created by TechMaster on 9/24/2016.
 */
public class TrollAutonomousRedBeacon extends TrollAutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
//Red Team
        map();

        moveForwardPID(.5, 180); //move forward off wall

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
        //Color sensor and button pusher code would go here eventually

        while (!middleLine()) {  //move forward again to the next beacon
            startMotors(.8, .8);
        }

        //Robot should now be in correct position ready to press beacon
        //Color sensor and button pusher code would go here eventually

        stopMotors();

    }
}
