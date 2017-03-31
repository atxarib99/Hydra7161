package org.firstinspires.ftc.teamcode.Lernaean.DeprecatedFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//@Autonomous(name = "BlueSecondary", group = "LinearOpMode")
@Deprecated
public class BlueSecondary extends LinearOpMode {

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

        boolean startProgram = false;
        boolean defenseBall = false;
        boolean defenseBeacons = false;
        boolean parkCorner = false;
        int ballsToShoot = 2;
        int delay = 0;
        boolean parkCenter = true;
        //wait for the program to actually start and display data in the meantime
        while(!startProgram) {
            if(gamepad1.start)
                startProgram = true;
            if(gamepad1.dpad_up) {
                ballsToShoot++;
                while(gamepad1.dpad_up);
            }
            if(gamepad1.dpad_down) {
                ballsToShoot--;
                while(gamepad1.dpad_down);
            }
            if(gamepad1.a) {
                parkCorner = !parkCorner;
                parkCenter = !parkCenter;
                defenseBall = false;
                defenseBeacons = false;
                while (gamepad1.a);
            }
            if(gamepad1.b) {
                parkCorner = false;
                parkCenter = false;
                defenseBall = true;
                defenseBeacons = false;
            }
            if(gamepad1.x){
               parkCorner = false;
               parkCenter = false;
               defenseBall = false;
               defenseBeacons = true;
            }
            if(gamepad1.y) {
                delay += 1000;
                while(gamepad1.y);
            }

            telemetry.addData("Instructions", "Press x to increase the delay by one second");
            telemetry.addData("Instructions", "Use D-Pad to change shooting");
            telemetry.addData("Instructions", "Press A to toggle parking between center and ramp");
            telemetry.addData("Instructions", "Press Start when finished");
            telemetry.addData("Instructions", "Press B to run a defense auto with center");
            telemetry.addData("Instructions", "Press X to go straight to their beacons");

            telemetry.addData("Balls to shoot", ballsToShoot);

            telemetry.addData("Delay", delay);

            if(parkCenter)
                telemetry.addData("Parking", "Center and Cap Ball");
            else if(parkCorner)
                telemetry.addData("Parking", "Corner Ramp");
            else if(defenseBall)
                telemetry.addData("Defense", "Center");
            else
                telemetry.addData("Defense", "Beacons");

            telemetry.update();
            idle();
        }

        while(!opModeIsActive()) {
            telemetry.addData("Balls to shoot", ballsToShoot);
            if(parkCenter)
                telemetry.addData("Parking", "Center and Cap Ball");
            else
                telemetry.addData("Parking", "Corner Ramp");

            telemetry.update();
            idle();
        }

        //wait for the program to actually start in the meantime just display data
        while (!opModeIsActive()) {
            telemetry.update();
            idle();
        }

        resetStartTime();

        //start the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display the current step
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        Thread.sleep(delay);

        //move to shooting range
        drivetrain.moveBackward(.35, 4744, 5000);

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

        //let the shooter run for 1 second
        Thread.sleep(900);

        if(ballsToShoot > 1) {
            //stop the balls from moving
            manipulator.runCollector(0);

            //wait for spinup
            Thread.sleep(400);

            //run the rest of the balls for the rest of the time
            manipulator.runCollector(-1);

            Thread.sleep(1500);

            //display that we are gonna start our rotation
            telemetry.addData("currentStep", "rotating");
            telemetry.update();

            //stop the collector
            manipulator.runCollector(0);
        }

        shooter.stopShooter();


        if(defenseBall) {
            drivetrain.rotatePB(.75, -42);

            drivetrain.stopMotors();

            Thread.sleep(5000);

            drivetrain.moveForward(1, 1, 4167, 5000);

            Thread.sleep(250);

            drivetrain.moveForward(-1, 0, 833, 2000);

            Thread.sleep(250);

            drivetrain.stopMotors();
        }

        if(defenseBeacons) {
            manipulator.runCollector(0);

            drivetrain.rotatePB(.75, -80);

            drivetrain.stopMotors();

            Thread.sleep(3500);

            drivetrain.moveForward(1, 1, 5858, 5000);

            drivetrain.rotatePDefense(.5, 20);

            drivetrain.stopMotors();

            Thread.sleep(250);

            drivetrain.moveForward(1, 1, 5500, 5000);

            drivetrain.rotateP(.75, 38);

            drivetrain.stopMotors();

            Thread.sleep(250);

            drivetrain.moveForward(1, 4167, 5000);
        }

        if(parkCorner) {
            // write some code here
        }

        if(parkCenter) {
            Thread.sleep(1750);

            //stop the shooter
            shooter.stopShooter();

            manipulator.runCollector(0);

            drivetrain.moveBackward(.2, 3300, 5000);

            drivetrain.moveForward(0, 1, 833, 2000);
        }

    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("AVg", new Func<String>() {
                    @Override
                    public String value() {
                        return "avg: " + drivetrain.getEncoderAvg();
                    }
                });
        telemetry.addLine()
                .addData("ods", new Func<String>() {
                    @Override
                    public String value() {
                        return "ods: " + drivetrain.sensor.leftODS() + " " + drivetrain.sensor.rightODS();
                    }
                });
        telemetry.addLine()
                .addData("gyro", new Func<String>() {
                    @Override
                    public String value() {
                        return "gyro: " + drivetrain.sensor.getGyroYaw();
                    }
                });
        telemetry.addLine()
                .addData("motorLPower", new Func<String>() {
                    @Override
                    public String value() {
                        return "leftPower: " + drivetrain.motorBL.getPower();
                    }
                });
        telemetry.addLine()
                .addData("motorRPower", new Func<String>() {
                    @Override
                    public String value() {
                        return "rightPower: " + drivetrain.motorBR.getPower();
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override
                    public String value() {
                        return "Color: " + beaconPushers.getColorVal();
                    }
                });
    }
}

