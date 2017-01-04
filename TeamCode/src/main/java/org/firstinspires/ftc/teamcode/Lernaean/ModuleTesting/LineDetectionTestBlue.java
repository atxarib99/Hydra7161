package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import android.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
<<<<<<< HEAD
=======
import org.firstinspires.ftc.teamcode.Libraries.Lift;
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
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
<<<<<<< HEAD
=======
    private Lift lift;
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
    private double voltage;

    private String version;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
<<<<<<< HEAD
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        version = "1.34";
=======
        lift = new Lift(this);
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        version = "1.37";
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        drivetrain.resetEncoders();

        waitForStart();

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

        drivetrain.rotatePB(.5, -137);

        drivetrain.stopMotors();

        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(100);

        drivetrain.setNullValue();

        telemetry.addData("currentStep", "movingForward");
        telemetry.update();

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

<<<<<<< HEAD
        drivetrain.moveBackwardToWall(-.35, 4000);
=======
        drivetrain.moveBackwardToWall(-.6, 4000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();

        telemetry.addData("currentStep", "turning back");

        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        Thread.sleep(100);

<<<<<<< HEAD
        drivetrain.rotatePZeroRevB(.35);
=======
        drivetrain.rotatePZeroRevB(.6);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

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

<<<<<<< HEAD
        drivetrain.moveFowardToLine(-.15, -.3);
=======
        drivetrain.moveFowardToLine(-.2, -.5);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();

        Thread.sleep(100);

<<<<<<< HEAD
        drivetrain.moveFowardToLine(.1, .13, 3000);
=======
        drivetrain.moveFowardToLine(.22, .27, 3000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();

        if (!beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
            beaconPushers.frontPush();
        }

        drivetrain.setNullValue();

<<<<<<< HEAD
        drivetrain.moveBackward(.5, 500, 500);
=======
        drivetrain.moveBackward(.7, 500, 500);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        Thread.sleep(100);

        drivetrain.stopMotors();

        Thread.sleep(250);

<<<<<<< HEAD
        drivetrain.moveFowardToLine(.17, .25, 5000);  //This one corrects for drift but we are accurate with it
=======
        drivetrain.moveFowardToLine(.25, .37, 5000);  //This one corrects for drift but we are accurate with it
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();

        Thread.sleep(250);

<<<<<<< HEAD
        drivetrain.moveFowardToLine(-.12, -.17, 3000); //move back to be aligned with white line
=======
        drivetrain.moveFowardToLine(-.15, -.20, 3000); //move back to be aligned with white line
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();

        Thread.sleep(250);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        if (beaconPushers.isBackRed()){
            beaconPushers.frontPush();
        }
        else {
            beaconPushers.backPush();
        }

        drivetrain.stopMotors();

<<<<<<< HEAD
        drivetrain.moveForward(-.35, 1000, 1000);
=======
        drivetrain.moveForward(-.6, 1000, 1000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        while(Math.abs(drivetrain.sensor.getGyroYaw()) > 85) {
            drivetrain.startMotors(-.75, 0);
        }
        drivetrain.stopMotors();

<<<<<<< HEAD
        drivetrain.moveBackward(-.75, 3000, 5000);
=======
        drivetrain.moveBackward(-1, 3000, 5000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.

        drivetrain.stopMotors();
    }
}
