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
@Autonomous(name = "ShootAndPush", group = "LinearOpMode")
public class ShootAndPushBall extends LinearOpMode {

    //create robot objects
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;
    private Lift lift;

    //create class specific variables
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

        //start the displaying of data
        composeTelemetry();

        //wait 2 seconds for voltage to pick back up after init
        Thread.sleep(2000);

        //calculate voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

         /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.43";

        //display the data to the user
        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        //reset the encoders
        drivetrain.resetEncoders();

        //wait for the program to actually start in the meantime just display data
        while(!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        //start the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display the current step
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //move to shooting range
        drivetrain.moveForward(.35, 4000, 5000);

        //display that we are going to shoot
        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        //turn the safe off
        manipulator.activateShooter();

        //move the arms out of they way of the wheels
        lift.openArms();

        Thread.sleep(10);

        //move the top grabbing mechanism out of the way of the balls path
        lift.topGrab();

        //start the shooter at the calculated power from the voltage value saved
        shooter.startShooter(-shooter.getNeededPower(voltage));

        //wait one second for the shooter to spin-up
        Thread.sleep(1000);

        //start moving the collecter
        manipulator.runCollector(-1);

        //let the shooter run for 1 second
        Thread.sleep(900);

        manipulator.runCollector(0);

        //wait for spinup
        Thread.sleep(400);

        //let the rest of the balls go
        manipulator.runCollector(-1);

        Thread.sleep(1750);

        //stop the shooter
        shooter.stopShooter();

        //put the top grabbing mechanism back into resting postion
        lift.topUngrab();

        //put the arms back into resting postion
        lift.grabArms();

        //wait 10 seconds so that we dont run the cap ball into other teams
        Thread.sleep(10000);

        //move onto the capballs spot and park there
        drivetrain.moveBackward(.2, 5500, 5000);

        //saftey stop for the programs
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