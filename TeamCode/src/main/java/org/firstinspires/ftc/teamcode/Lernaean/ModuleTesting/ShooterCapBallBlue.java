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
@Autonomous(name = "ShooterCapBallBlue", group = "LinearOpMode")
public class ShooterCapBallBlue extends LinearOpMode {

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

        version = "1.000";

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

        drivetrain.moveForward(.5, 3000, 5000);

        drivetrain.stopMotors();

        drivetrain.rotateP(.5, -60);
    }
}