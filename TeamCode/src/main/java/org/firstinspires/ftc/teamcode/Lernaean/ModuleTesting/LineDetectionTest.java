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
        voltage = hardwareMap.voltageSensor.get("Motor Controller 4").getVoltage();

        version = "1.96";

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        waitForStart();

        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        drivetrain.setNullValue();

        drivetrain.moveForward(.25, (int) (.15 * 1120), 2000);

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

        drivetrain.rotateP(.75, -42); /// 31 or so if going for first line

        drivetrain.stopMotors();

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(500);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.moveForward(.25, 3750, 5000);

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "turning back");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(500);

        drivetrain.rotatePZero(.5);

        drivetrain.stopMotors();

        drivetrain.moveForward(.25, 250, 1000);

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        drivetrain.moveFowardToLine(-.2, -.2);

        if (beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
            beaconPushers.frontPush();
        }

        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

        Thread.sleep(2000);

//        while(!drivetrain.sensor.isLeftLine()) {
//            drivetrain.startMotors(.3, .3);
//            idle();
//        }

        drivetrain.setNullValue();

//        drivetrain.moveForward(.25, 250);

        drivetrain.stopMotors();

        drivetrain.rotatePZeroRev(.5);

        drivetrain.stopMotors();

        drivetrain.moveFowardToLine(.27, .2);  //This one corrects for drift but we are accurate with it

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(-.2, -.2); //move back to be aligned with white line

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.rotatePZero(.5);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        if (beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
            beaconPushers.frontPush();
        }
    }
}
