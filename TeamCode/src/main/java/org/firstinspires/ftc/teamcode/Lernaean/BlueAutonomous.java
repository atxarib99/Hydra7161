package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class BlueAutonomous extends LinearOpMode {

    //create class variables
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

        //display the values
        composeTelemetry();

        //wait 2 seconds to regain voltage dropped from init
        Thread.sleep(2000);

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.43";

        //display the data for testing purposes
        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        //reset the encoders
        drivetrain.resetEncoders();

        //wait for the program to actually start and display data in the meantime
        while(!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        //start gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display current step
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //move forward to get into shooting range
        drivetrain.moveForward(.35, 2000, 5000);

        //display that we are going to shoot
        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        //turn the safe off
        manipulator.activateShooter();

        //move the arms out of the way
        lift.openArms();

        idle();

        //move the top grabber mechanism out of the way
        lift.topGrab();

        //start the shooter at the calculated power from the voltage value saved
        shooter.startShooter(-shooter.getNeededPower(voltage));

        //wait one second for the shooter to spin-up
        Thread.sleep(1000);

        //start moving the collecter
        manipulator.runCollector(-1);

        //let the shooter run for 1 seconds
        Thread.sleep(900);

        //stop the balls from moving
        manipulator.runCollector(0);

        //wait for spinup
        Thread.sleep(400);

        //run the rest of the balls for the rest of the time
        manipulator.runCollector(-1);

        Thread.sleep(1750);

        //display that we are gonna start our rotation
        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        //stop the shooter
        shooter.stopShooter();

        //move the top grabber mechanism into its rest postion
        lift.topUngrab();

        //move the arms into their rest position
        lift.grabArms();

        //stop the collector
        manipulator.runCollector(0);

        //move away from shooting zone
        drivetrain.moveForward(-.35, 1000, 5000);

        //wait for momentum
        Thread.sleep(100);

        //turn PID
        drivetrain.rotatePB(.4, -142);

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait for momentum
        Thread.sleep(250);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        manipulator.activateShooter(false);

        //run the collector in reverse to push balls out of the way
        manipulator.runCollector(.5);

        //
        drivetrain.moveBackwardToWall(-1, -.4, 11500, 10000, 142);

        //stop moving the collector
        manipulator.runCollector(0);

        telemetry.addData("currentStep", "turning back");
        //wait for momentum
        Thread.sleep(100);

        telemetry.addData("currentStep", "finding the whiteline");
        telemetry.update();

        //move towards the beacons at a high correction
        drivetrain.moveForward(-.2, -.35, 4000, 5000);

        //slow down and detect the line
        drivetrain.moveFowardToLine(-.09, -.12, 4000);

        //wait for momentum
        Thread.sleep(100);

        //Press the beacon 2 times and on the third time correct a bit before the last push
        int count = 0;
        while (!beaconPushers.areBothBlue()) {
            if(count == 2) {
                drivetrain.moveForward(.08, .11, 100, 500);
            }
            if (beaconPushers.isBackBlue()){
                beaconPushers.backPush();
            }
            else {
                beaconPushers.frontPush();
            }
            if(count == 2)
                break;
            count++;
        }

        if(beaconPushers.areBothRed()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        //move fast towards the next beacon
        drivetrain.moveForward(.3, .7, 5000, 5000);

        //move towards the line at a high speed
        drivetrain.moveFowardToLine(.14, .23, 2000);

        //wait for momentum
        Thread.sleep(250);

        //correct to line
        drivetrain.moveFowardToLine(-.09, -.11, 3000); //move back to be aligned with white line

        //wait for momentum
        Thread.sleep(250);

        //Press the beacon 2 times and on the third time correct a bit before the last push
        while (!beaconPushers.areBothBlue()) {
            if(count == 2) {
                drivetrain.moveForward(.08, .11, 100, 500);
            }
            if (beaconPushers.isBackBlue()){
                beaconPushers.backPush();
            }
            else {
                beaconPushers.frontPush();
            }
            if(count == 2)
                break;
            count++;
        }

        //make sure we did not hit the wrong color
        if(beaconPushers.areBothRed()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        //move forward a bit
        drivetrain.moveForward(-.75, 1000, 1000);

        //turn away from the wall
        while(opModeIsActive() && Math.abs(drivetrain.sensor.getGyroYaw()) > 85) {
            drivetrain.startMotors(-.65, 0);
            idle();
        }
        drivetrain.stopMotors();

        //move to the center zone push and park
        drivetrain.moveBackward(-1, 6000, 5000);

        //safety stop for program
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