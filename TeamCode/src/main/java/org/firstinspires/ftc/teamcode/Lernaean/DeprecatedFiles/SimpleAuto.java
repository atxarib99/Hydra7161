package org.firstinspires.ftc.teamcode.Lernaean.DeprecatedFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/*
 * Created by Arib on 10/17/2016.
 */
@Autonomous(name = "slow forward", group = "LinearOpMode")
@Disabled
@Deprecated
public class SimpleAuto extends LinearOpMode {

    Drivetrain drivetrain;
    BeaconPushers beaconPushers;
    Manipulator manipulator;
    Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        beaconPushers = new BeaconPushers(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);

        while (opModeIsActive())
            drivetrain.startMotors(.2, .2);

        drivetrain.stopMotors();
    }
}
