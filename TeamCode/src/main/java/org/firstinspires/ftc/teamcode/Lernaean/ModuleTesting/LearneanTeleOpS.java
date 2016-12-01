package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 11/23/2016.
 */

public class LearneanTeleOpS extends LearnaenLinearOpMode {

    boolean isbackOut = false;
    boolean isfrontOut = false;


    @Override
    public void runOpMode() throws InterruptedException {
        //initialize the robot
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad2.right_bumper) {
                shooter.startShooter(shooterPower);
            }

            if(gamepad2.x) {
                manipulator.activateShooter(true);
            }

            if(gamepad2.b) {
                manipulator.activateShooter(false);
            }

            if(gamepad2.dpad_up) {
                while(gamepad2.dpad_up);
                shooterPower += .05;
            }

            if(gamepad2.dpad_down) {
                while(gamepad2.dpad_down);
                shooterPower -= .05;
            }

            if(gamepad2.left_bumper)
                shooter.stopShooter();

            if(gamepad2.a){
                while(gamepad2.a);
                if (!isbackOut) {
                    beaconPushers.backOut(true);
                    isbackOut = true;
                } else {
                    beaconPushers.backOut(false);
                    isbackOut = false;
                }
            }

            if(gamepad2.y){
                while(gamepad2.y);
                if(!isfrontOut){
                    beaconPushers.backOut(true);
                    isfrontOut = true;
                } else {
                    beaconPushers.backOut(false);
                    isfrontOut = false;
                }
            }

            if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_y) > .05)){
                drivetrain.startMotors(gamepad1.right_stick_y, gamepad1.left_stick_y);
            } else {
                drivetrain.stopMotors();
            }

            if(gamepad2.right_trigger > .05 && gamepad2.left_trigger > .05) {
                manipulator.runCollector(-.5);
            } else if(gamepad2.right_trigger > .05) {
                manipulator.runCollector(1);
            } else if(gamepad2.left_trigger > .05)
                manipulator.runCollector(0);

            if(gamepad1.a) {
                while(gamepad1.a);
                drivetrain.reverse();
            }
        }
    }


}
