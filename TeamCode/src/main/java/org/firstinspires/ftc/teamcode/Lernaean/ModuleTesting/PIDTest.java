package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 10/28/2016.
 */
@Autonomous(name = "PIDTest", group = "LinearOpMode")
public class PIDTest extends LinearOpMode {

    Drivetrain drivetrain;
    Manipulator manipulator;
    Shooter shooter;
    BeaconPushers beaconPushers;

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);

        version = "1.1";

        telemetry.addData("version: ", version);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        while(opModeIsActive()) {
            drivetrain.sensor.resetGyro();
            telemetry.addData("runMode", "ready");
            telemetry.update();
            while(!gamepad1.a);
            telemetry.addData("runMode", "running");
            drivetrain.rotatePReset(.5, 90);
            idle();
        }
    }
}
