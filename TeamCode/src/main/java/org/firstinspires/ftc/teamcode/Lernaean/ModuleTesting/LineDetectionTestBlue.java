package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Lift;
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
    private Lift lift;
    private double voltage;

    private String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        lift = new Lift(this);
        composeTelemetry();
        Thread.sleep(2000);
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        version = "1.39";

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        drivetrain.resetEncoders();

        while(!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        drivetrain.setNullValue();

        drivetrain.moveForward(.35, (int) (.15 * 1120), 5000);

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        //turn the safe off
        manipulator.activateShooter();

        //start the shooter at the calculated power from the voltage value saved
        shooter.startShooter(-shooter.getNeededPower(voltage));

        //wait one second for the shooter to spin-up
        Thread.sleep(1000);

        //start moving the collecter
        manipulator.runCollector(-1);

        //let the shooter run for 3 seconds
        Thread.sleep(500);

        manipulator.runCollector(0);

        Thread.sleep(500);

        manipulator.runCollector(-1);

        Thread.sleep(1500);

        //display that we are gonna start our rotation
        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        //stop the shooter
        shooter.stopShooter();

        //stop the collector
        manipulator.runCollector(0);

        Thread.sleep(100);

        drivetrain.rotatePB(.5, -147);

        drivetrain.stopMotors();

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(100);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.moveBackwardToWall(-.3, 5000);

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "turning back");

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(100);

        drivetrain.rotatePZeroRevB(.35);

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

        Thread.sleep(250);

        drivetrain.moveForward(-.25, 500, 1000);

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(.17, .29, 1500);

        drivetrain.stopMotors();

        Thread.sleep(100);

        drivetrain.moveFowardToLine(-.11, -.13, 3000);

        drivetrain.stopMotors();

        if (beaconPushers.isBackBlue()){
                beaconPushers.backPush();
        }
        else {
                beaconPushers.frontPush();
        }

        drivetrain.setNullValue();

        drivetrain.moveBackward(.3, 500, 500);

        Thread.sleep(100);

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(.17, .27, 2000);  //This one corrects for drift but we are accurate with it

        drivetrain.stopMotors();

        Thread.sleep(250);

        drivetrain.moveFowardToLine(-.11, -.13, 3000); //move back to be aligned with white line

        drivetrain.stopMotors();

        Thread.sleep(250);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        if (beaconPushers.isBackBlue()){
                beaconPushers.backPush();
        }
        else {
                beaconPushers.frontPush();
        }

        drivetrain.stopMotors();

        drivetrain.moveForward(-.6, 1000, 1000);

        while(Math.abs(drivetrain.sensor.getGyroYaw()) > 85) {
            if(voltage > 14)
                drivetrain.startMotors(-.65, 0);
            else
                drivetrain.startMotors(-.75, 0);
            idle();
        }
        drivetrain.stopMotors();

        drivetrain.moveBackward(-.75, 3000, 5000);

        drivetrain.stopMotors();
    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("AVg", new Func<String>() {
                    @Override public String value() {
                        return "avg: " + drivetrain.getEncoderAvg();
                    }
                });
        telemetry.addLine()
                .addData("ods", new Func<String>() {
                    @Override public String value() {
                        return "ods: " + drivetrain.sensor.leftODS() + " " + drivetrain.sensor.rightODS();
                    }
                });
        telemetry.addLine()
                .addData("gyro", new Func<String>() {
                    @Override public String value() {
                        return "gyro: " + drivetrain.sensor.getGyroYaw();
                    }
                });
        telemetry.addLine()
                .addData("motorLPower", new Func<String>() {
                    @Override public String value() {
                        return "leftPower: " + drivetrain.motorBL.getPower();
                    }
                });
        telemetry.addLine()
                .addData("motorRPower", new Func<String>() {
                    @Override public String value() {
                        return "rightPower: " + drivetrain.motorBR.getPower();
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override public String value() {
                        return "Color: " + beaconPushers.getColorVal();
                    }
                });
    }
}
