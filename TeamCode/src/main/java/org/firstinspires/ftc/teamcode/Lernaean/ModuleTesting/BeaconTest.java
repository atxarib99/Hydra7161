package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 10/20/2016.
 */
@Autonomous(name = "BeaconTest", group = "Testing")
public class BeaconTest extends LinearOpMode {

    BeaconPushers beaconPushers;

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        beaconPushers   = new BeaconPushers(this);

        version = "1.3";

        telemetry.addData("version", version);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            boolean blue = beaconPushers.isBackBlue();
            telemetry.addData("color", beaconPushers);
            String toTele;
            if(!blue) {
                toTele = "The back sensor detects red and the front is blank";
            }
            else {
                toTele = "The front sensor detects not red and the front is blank";
            }

            telemetry.addData("beacon", beaconPushers.getColorVal());
            telemetry.update();

            Thread.sleep(2000);

            blue = beaconPushers.isBackBlue();
            int count = 0;
            boolean attempted = false;
            while (beaconPushers.isBeaconUnpressed()) {
                if (blue) {
                    beaconPushers.backPush();
                    attempted = true;
                }
                else {
                    beaconPushers.frontPush();
                    attempted = true;
                }
                if(count == 2)
                    break;
                count++;
                Thread.sleep(250);
            }

        }
    }
}
