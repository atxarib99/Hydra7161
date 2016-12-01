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
@Autonomous(name = "Blue Autonomous", group = "LinearOpMode")
public class LineDetectionTestBlue extends LinearOpMode {

    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;
    private double voltage;

    private String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        version = "1.31";

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        drivetrain.resetEncoders();

        waitForStart();

        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        drivetrain.setNullValue();

        drivetrain.moveBackward(-.25, (int) (.15 * 1120), 5000);

        drivetrain.stopMotors();

        drivetrain.setNullValue();


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

        Thread.sleep(100);

        drivetrain.rotatePB(.75, 39);

        drivetrain.stopMotors();

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(250);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.moveBackward(-.35, 4700, 5000);

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "turning back");

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(100);

        drivetrain.rotatePZeroRevB(.45);

        drivetrain.stopMotors();

        Thread.sleep(100);

        drivetrain.stopMotors();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.setNullValue();

//        drivetrain.moveForward(-.5, 500, 1000);

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

//        while(!drivetrain.sensor.isLeftLine()) {
//            drivetrain.startMotors(.3, .3);
//            idle();
//        }

        drivetrain.moveFowardToLine(.2, .2, 3000);

        drivetrain.stopMotors();

        Thread.sleep(100);

        drivetrain.moveFowardToLine(-.15, -.15, 3000);

        drivetrain.stopMotors();

        if (!beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
            beaconPushers.frontPush();
        }

        drivetrain.setNullValue();

        drivetrain.moveForward(-.5, 100, 500);

        Thread.sleep(100);

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(-.2, -.21);  //This one corrects for drift but we are accurate with it

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(.15, .15, 4000); //move back to be aligned with white line

        drivetrain.stopMotors();

        Thread.sleep(250);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        if (beaconPushers.isBackRed()){
            beaconPushers.frontPush();
        }
        else {
            beaconPushers.backPush();
        }

        drivetrain.rotatePB(1, 35);

        drivetrain.setNullValue();

        drivetrain.moveForward(.5, 1000, 3000);

        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        manipulator.activateShooter();

        shooter.startShooter(-shooter.getNeededPower(voltage));

        Thread.sleep(1000);

        manipulator.runCollector(-.5);

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
    }
}
