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
 * Created by Arib on 2/23/2017.
 */

@Autonomous(name = "RedSimpleAuto", group = "RedAuto")
public class RedAutonomousSimpleColor extends LinearOpMode {

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

        //create telemetry data
        composeTelemetry();

        //wait two seconds to regain voltage drop from init
        Thread.sleep(2000);

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

        //wait for autonmous to actually start in the meantime return values
        while(!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        //start the acceleration calculation for the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display that we are moving off the wall
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //move forward to get within shooting range
        drivetrain.moveBackward(.35, 2000, 5000);

        //display that we are going to shoot
        telemetry.addData("currentStep", "shooting");
        telemetry.update();

        //turn the safe off
        manipulator.activateShooter();

        //move the top grabber out of shooting path
        lift.topGrab();

        //move the arms away from the shooters
        lift.openArms();

        //start the shooter at the calculated power from the voltage value saved
        shooter.startShooter(-shooter.getNeededPower(voltage));

        //wait one second for the shooter to spin-up
        Thread.sleep(1000);

        //start moving the collector
        manipulator.runCollector(-1);

        //let the balls move for 1 second
        Thread.sleep(1000);

        //stop the balls from moving
        manipulator.runCollector(0);

        //wait 1/2 second to wait for spinup
        Thread.sleep(500);

        //let the balls move again
        manipulator.runCollector(-1);

        //keep the balls moving for 1.5 seconds
        Thread.sleep(1500);

        //stop the shooter
        shooter.stopShooter();

        //stop the collector
        manipulator.runCollector(0);

        //move the top grabbing mechanism back to rest position
        lift.topUngrab();

        //move the arms back to rest position
        lift.grabArms();

        //move away from the shooting zone
        drivetrain.moveBackward(-.15, 1000, 5000);

        //rotate 38 degrees to the left
        drivetrain.rotateP(.4, -38);

        //stop after the rotation safety stop
        drivetrain.stopMotors();

        //wait 1/2 seconds for momentum
        Thread.sleep(500);

        //display that we are going to move forward
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward to the wall
        drivetrain.moveForwardToWall(1, .3, 11000, 10000, 38);

        //move forward into line
        drivetrain.moveFowardToLine(.13, .18, 4000);

        //wait for momentum
        Thread.sleep(250);

        //correct back onto the line
        drivetrain.moveFowardToLine(-.09, -.12, 5000);

        if (beaconPushers.isBackBlue()){
            beaconPushers.frontPush();
        }
        else {
            beaconPushers.backPush();
        }

        //make sure we didn't hit the wrong color
        if(beaconPushers.areBothBlue()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        //display we are moving forwards
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward at high speed towards the next beacon
        drivetrain.moveForward(-.3, -.7, 6000, 5000);

        //slow down while finding the line
        drivetrain.moveFowardToLine(-.09, -.12, 5000);

        //wait for momentum
        Thread.sleep(200);

        if (beaconPushers.isBackBlue()){
            beaconPushers.frontPush();
        }
        else {
            beaconPushers.backPush();
        }
        //make sure we didnt hit the wrong color
        if(beaconPushers.areBothBlue()) {
            Thread.sleep(5000);
            beaconPushers.backPush();
            beaconPushers.frontPush();
        }

        //move forward a bit
        drivetrain.moveForward(.6, 2000, 1000);

        //turn off the wall and onto the cap ball
        try {
            while(opModeIsActive() && drivetrain.sensor.getGyroYaw() < 100) {
                drivetrain.startMotors(.6, 0);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //stop the motors
        drivetrain.stopMotors();

        //move to push capball off and push
        drivetrain.moveForward(.5, 6000, 5000);

        //saftey stop for end of program
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
