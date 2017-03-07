package org.firstinspires.ftc.teamcode.Lernaean;

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
@Autonomous(name = "RedAutonomousLow", group = "LinearOpMode")
public class RedAutonomousLow extends LinearOpMode {
    //Create robot objects
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;
    private Lift lift;

    //create local variables

    private double voltage;
    private String version;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize the robot
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
        lift = new Lift(this);

        composeTelemetry();

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.136";

        //display the voltage and version for testing
        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        //wait for autonmous to actually start
        while(!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        //start the acceleration calculation for the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display that we are moving off the wall
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //soft reset the encoders
        drivetrain.setNullValue();

        drivetrain.moveBackward(.35, 2000, 5000);

        //soft reset the encoders
        drivetrain.setNullValue();

        //display that we are going to shoot
        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        //turn the safe off
        manipulator.activateShooter();

        //start the shooter at the calculated power from the voltage value saved
        shooter.startShooter(-shooter.getNeededPower(voltage));

        //wait one second for the shooter to spin-up
        Thread.sleep(1000);

        //start moving the collecter
        manipulator.runCollector(-1);

        //let the shooter run for 3 seconds
        Thread.sleep(1000);

        manipulator.runCollector(0);

        Thread.sleep(400);

        manipulator.runCollector(-1);

        Thread.sleep(1500);

        //display that we are gonna start our rotation
        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        //stop the shooter
        shooter.stopShooter();

        //stop the collector
        manipulator.runCollector(0);

        drivetrain.moveBackward(-.15, 1000, 5000);

        //rotate 42 degrees to the left
        drivetrain.rotateP(.6, -30); /// 31 or so if going for first line

        //stop after the rotation
        drivetrain.stopMotors();

        Thread.sleep(500);

        //show the current angle for testing purposes
        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait a bit for the momentum to stop moving the robot
        Thread.sleep(100);

        //reset the encoder
        drivetrain.setNullValue();

        //display that we are going to move forward
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward to the wall
        drivetrain.moveForwardToWall(1, .3, 12000, 10000, 30);

        drivetrain.moveFowardToLine(.15, .2, 4500);

        Thread.sleep(250);

        drivetrain.moveFowardToLine(-.1, -.13, 5000);

        int count = 0;
        while (!beaconPushers.areBothRed()) {
            if(count == 3) {
                drivetrain.moveForward(.1, .12, 250, 500);
            }
            if (beaconPushers.isBackBlue()){
                beaconPushers.frontPush();
            }
            else {
                beaconPushers.backPush();
            }
            if(count == 3)
                break;
            count++;
        }

        if(beaconPushers.areBothBlue()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        //display we are moving forwards
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

        Thread.sleep(100);

//        while(!drivetrain.sensor.isLeftLine()) {
//            drivetrain.startMotors(.3, .3);
//            idle();
//        }

        drivetrain.setNullValue();

        drivetrain.moveForward(-.5, -.7, 6000, 5000);

        drivetrain.moveFowardToLine(-.1, -.13, 5000);

        Thread.sleep(200);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        count = 0;
        while (!beaconPushers.areBothRed()) {
            if(count == 3) {
                drivetrain.moveForward(.1, .12, 250, 500);
            }
            if (beaconPushers.isBackBlue()){
                beaconPushers.frontPush();
            }
            else {
                beaconPushers.backPush();
            }
            if(count == 3)
                break;
            count++;
        }

        if(beaconPushers.areBothBlue()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        drivetrain.moveForward(.6, 2000, 1000);

        try {
            while(drivetrain.sensor.getGyroYaw() < 100) {
                drivetrain.startMotors(.6, 0);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        drivetrain.stopMotors();

        drivetrain.setNullValue();

        drivetrain.moveForward(.5, 6000, 5000);

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
