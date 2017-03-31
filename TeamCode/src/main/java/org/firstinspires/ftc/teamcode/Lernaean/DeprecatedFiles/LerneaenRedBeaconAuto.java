package org.firstinspires.ftc.teamcode.Lernaean.DeprecatedFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 9/14/2016.
 */
//@Autonomous(name="LernaeanRedBeacon", group="Linear Opmode")
@Deprecated
public class LerneaenRedBeaconAuto extends LinearOpMode {

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

//        drivetrain.moveForward(-.5, (int) (.477 * 1120));

        drivetrain.setNullValue();

        drivetrain.rotateP(.5, -45);

        drivetrain.setNullValue();

//        drivetrain.moveForward(-.8, (int) (4.77 * 1120)); //4.77

        while(!drivetrain.sensor.isLeftLine())
            drivetrain.startMotors(-.4, -.4);

        drivetrain.stopMotors();

        while(!drivetrain.sensor.isRightLine())
            drivetrain.startMotors(-.4, 0);

        drivetrain.stopMotors();

        boolean moveBack = beaconPushers.isBackBlue();

        if(moveBack) {
            beaconPushers.backPush();
        } else {
            beaconPushers.frontPush();
        }

        drivetrain.setNullValue();

//        drivetrain.moveForward(-.8, (int) (3.1 * 1120));

        drivetrain.stopMotors();

        while(!drivetrain.sensor.isLeftLine())
            drivetrain.startMotors(-.4, -.4);

        drivetrain.stopMotors();

        while(!drivetrain.sensor.isRightLine())
            drivetrain.startMotors(-.4, 0);

        drivetrain.stopMotors();

        moveBack = beaconPushers.isBackBlue();

        if(moveBack) {
            beaconPushers.backPush();
        } else {
            beaconPushers.frontPush();
        }

    }
}
