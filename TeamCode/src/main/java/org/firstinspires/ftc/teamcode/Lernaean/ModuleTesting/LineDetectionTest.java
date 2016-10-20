package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Sensor;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 10/20/2016.
 */
@Autonomous(name = "LineDetectionTest", group = "LinearOpMode")
public class LineDetectionTest extends LinearOpMode {

    Drivetrain drivetrain;
    Manipulator manipulator;
    Shooter shooter;
    BeaconPushers beaconPushers;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);

        drivetrain.moveForward(.5, (int) (.477 * 1120));

        drivetrain.setNullValue();

        drivetrain.rotateP(.5, 45);

        drivetrain.setNullValue();

        drivetrain.moveForward(.8, (int) (4.77 * 1120)); //4.77

        while(!drivetrain.sensor.isLeftLine())
            drivetrain.startMotors(.4, .4);

        drivetrain.stopMotors();

        while(!drivetrain.sensor.isRightLine())
            drivetrain.startMotors(.4, 0);

        drivetrain.stopMotors();
    }
}
