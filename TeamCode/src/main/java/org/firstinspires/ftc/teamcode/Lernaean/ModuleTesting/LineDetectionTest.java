package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import android.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
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
    double voltage;

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        voltage = hardwareMap.voltageSensor.get("Motor Controller 4").getVoltage();

        version = "1.59";

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        waitForStart();

        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        drivetrain.setNullValue();

        drivetrain.moveForward(.25, (int) (.15 * 1120));

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        manipulator.activateShooter();

        shooter.startShooter(-shooter.getNeededPower(voltage));

        Thread.sleep(1000);

        manipulator.runCollector(-1);

        Thread.sleep(3000);

        idle();
        beaconPushers.backOut(false);
        idle();
        beaconPushers.frontOut(false);
        idle();

        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        shooter.stopShooter();

        manipulator.runCollector(0);
//
//        drivetrain.moveForward(.5, 2000);
//
//        drivetrain.stopMotors();
//
//        drivetrain.rotateP(1, -45);
//
//        Thread.sleep(250);
//
//        drivetrain.rotateP(.5, 0);
//
//        Thread.sleep(250);
//
//        drivetrain.moveBackward(-.5, -2000);

        drivetrain.rotateP(.5, -50); /// 31 or so if going for first line

        drivetrain.stopMotors();

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(1000);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.moveForward(.3, 1250);

        drivetrain.stopMotors();

        drivetrain.rotatePReset(.5, 30);

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        Thread.sleep(1000);

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

//        while(!drivetrain.sensor.isLeftLine()) {
//            drivetrain.startMotors(.3, .3);
//            idle();
//        }

        drivetrain.moveFowardToLine(.3, .3);  //This one corrects for drift but we are accurate with it

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "finding the otherline");
        telemetry.update();

        while(!drivetrain.sensor.isRightLine()) {
            drivetrain.startMotors(.3, 0);
            idle();
        }

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "finding the firstline again");
        telemetry.update();

        while(!drivetrain.sensor.isLeftLine()) {
            drivetrain.startMotors(0, -.3);
            idle();
        }

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "finding the otherline");
        telemetry.update();

        while(!drivetrain.sensor.isRightLine()) {
            drivetrain.startMotors(.3, 0);
            idle();
        }

        while(!drivetrain.sensor.isLeftLine()) {
            drivetrain.startMotors(.25, .25);
            idle();
        }


        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();
    }
}
