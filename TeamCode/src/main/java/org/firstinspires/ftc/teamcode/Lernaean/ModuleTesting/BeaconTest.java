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
@Autonomous(name = "BeaconTest", group = "LinearOpMode")
public class BeaconTest extends LinearOpMode {

    Drivetrain drivetrain;
    Shooter shooter;
    BeaconPushers beaconPushers;
    Manipulator manipulator;

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain      = new Drivetrain(this);
        shooter         = new Shooter(this);
        beaconPushers   = new BeaconPushers(this);
        manipulator     = new Manipulator(this);

        version = "1.3";

        telemetry.addData("version", version);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            boolean backRed = beaconPushers.isBackRed();
            telemetry.addData("color", beaconPushers);
            String toTele;
            if(backRed) {
                toTele = "The back sensor detects red and the front is blank";
            }
            else {
                toTele = "The front sensor detects not red and the front is blank";
            }

            telemetry.addData("beacon", toTele);
            telemetry.update();

            Thread.sleep(2000);

            if(backRed) {
                beaconPushers.backPush();
            }
            else {
                beaconPushers.frontPush();
            }

            while (!gamepad1.a);

        }
    }
}
