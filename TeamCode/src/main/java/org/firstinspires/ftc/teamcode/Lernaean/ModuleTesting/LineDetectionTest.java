package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

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

    String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);

        version = "1.0";

        telemetry.addData("version: ", version);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        waitForStart();

        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        drivetrain.setNullValue();

        drivetrain.moveForward(.25, (int) (.225 * 1120));

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        drivetrain.rotateP(.5, -37);

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(1000);

        drivetrain.moveForward(.8, (int) (.77 * 1120)); //4.77

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

        while(!drivetrain.sensor.isRightLine())
            drivetrain.startMotors(.4, .4);

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "finding the otherline");
        telemetry.update();

        while(!drivetrain.sensor.isLeftLine())
            drivetrain.startMotors(.4, 0);

        telemetry.addData("currentStep", "finished");

        drivetrain.stopMotors();
    }
}
