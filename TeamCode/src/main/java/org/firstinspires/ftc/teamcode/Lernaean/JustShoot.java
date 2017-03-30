package org.firstinspires.ftc.teamcode.Lernaean;

import android.widget.ThemedSpinnerAdapter;

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
@Autonomous(name = "JustShoot", group = "LinearOpMode")
public class JustShoot extends LinearOpMode {

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

        version = "1.43";

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        drivetrain.resetEncoders();

        boolean startProgram = false;
        int ballsToShoot = 2;
        int position = 0;
        String[] afterShootingOptions = {"Cap Ball", "CornerVortex", "Defense", "Nothing", "Move Back"};
        String afterShooting = "Cap Ball";
        boolean red = true;
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
            if(gamepad1.dpad_right) {
                position++;
                position %= 5;
                afterShooting = afterShootingOptions[position];
                while(gamepad1.dpad_right);
            }
            if(gamepad1.dpad_left) {
                position--;
                if(position == -1)
                    position = 4;
                afterShooting = afterShootingOptions[position];
                while(gamepad1.dpad_left);
            }
            if(gamepad1.guide) {
                red = !red;
            }
            telemetry.addData("Instructions", "Use D-Pad to change shooting");
            telemetry.addData("Instructions", "Press D-Pad Right Left for after shooting options");
            telemetry.addData("Instructions", "Press the middle button to change side");
            telemetry.addData("Instructions", "Press Start when finished");

            telemetry.addData("Balls to shoot", ballsToShoot);
            telemetry.addData("AfterShooting", afterShooting);
            if(red) {
                telemetry.addData("Team", "Red");
            } else {
                telemetry.addData("Team", "Blue");
            }

            telemetry.update();
            idle();
        }

        while(!opModeIsActive()) {
            telemetry.addData("Balls to shoot", ballsToShoot);
            telemetry.addData("AfterShooting", afterShooting);
            if(red) {
                telemetry.addData("Team", "Red");
            } else {
                telemetry.addData("Team", "Blue");
            }
            telemetry.update();
            idle();
        }

        Thread.sleep(15000);

        //start the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display the current step
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //move to shooting range
        drivetrain.moveForward(.35, 5000, 5000);

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

        manipulator.runCollector(0);

        if(ballsToShoot > 1) {
            //wait for spinup
            Thread.sleep(600);

            //let the rest of the balls go
            manipulator.runCollector(-1);

            Thread.sleep(1750);

            manipulator.runCollector(0);
        }

        //stop the shooter
        shooter.stopShooter();

        //if we are red team
        if(red) {
            //if we are to push the Cap Ball
            if(afterShooting.equals(afterShootingOptions[0])) {
                drivetrain.moveBackward(.2, 3300, 5000);

                drivetrain.moveForward(0, 1, 833, 2000);
            }
            //if we are supposed to park on the corner vortex
            if(afterShooting.equals(afterShootingOptions[1])) {
                drivetrain.basicTurn(.25, 45);
            }
            //if we are supposed to play defense
            if(afterShooting.equals(afterShootingOptions[2])) {
                //TODO: DO RED DEFENSE CODE
            }
            //if we are to move back and out of the way
            if(afterShooting.equals(afterShootingOptions[4])) {
                drivetrain.moveBackward(-.35, 5000, 5000);
            }
        }
        else {
            //if we are to push the Cap Ball
            if(afterShooting.equals(afterShootingOptions[0])) {
                drivetrain.moveBackward(.2, 3300, 5000);

                drivetrain.moveForward(1, 0, 833, 2000);
            }
            //if we are supposed to park on the corner vortex
            if(afterShooting.equals(afterShootingOptions[1])) {
                drivetrain.basicTurn(.25, -45);
            }
            //if we are supposed to play defense
            if(afterShooting.equals(afterShootingOptions[2])) {
                //TODO: DO BLUE DEFENSE CODE
            }
            //if we are to move back and out of the way
            if(afterShooting.equals(afterShootingOptions[4])) {
                drivetrain.moveBackward(-.35, 5000, 5000);
            }
        }
        manipulator.runCollector(0);
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