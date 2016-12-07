package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import android.widget.ThemedSpinnerAdapter;

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
@Autonomous(name = "RedAutonomous", group = "LinearOpMode")
public class LineDetectionTest extends LinearOpMode {


    //Create robot objects
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;

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

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.126";

        //display the voltage and version for testing
        telemetry.addData("version: ", version);
        telemetry.addData("voltage", voltage);
        telemetry.addData("init", "init fully finished");
        telemetry.update();

        //wait for autonmous to actually start
        waitForStart();

        //start the acceleration calculation for the gyro
        drivetrain.sensor.gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //display that we are moving off the wall
        telemetry.addData("currentStep", "moving off the wall");
        telemetry.update();

        //soft reset the encoders
        drivetrain.setNullValue();

        //move forward .15 rotations this is enough off the wall
        drivetrain.moveForward(.35, (int) (.15 * 1120), 5000);

        //run a saftey stop command. the previous method has one but this ensures it
        drivetrain.stopMotors();

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
        Thread.sleep(3000);

        //display that we are gonna start our rotation
        telemetry.addData("currentStep", "rotating");
        telemetry.update();

        //stop the shooter
        shooter.stopShooter();

        //stop the collector
        manipulator.runCollector(0);

        //rotate 42 degrees to the left
        drivetrain.rotateP(.8, -37); /// 31 or so if going for first line

        //stop after the rotation
        drivetrain.stopMotors();

        //show the current angle for testing purposes
        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait a bit for the momentum to stop moving the robot
        Thread.sleep(50);

        //reset the encoder
        drivetrain.setNullValue();

        //display that we are going to move forward
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward to the wall
        drivetrain.moveForwardToWall(.75, 5000);

        //safety stop
        drivetrain.stopMotors();

        //display that we are going to even out with the wall
        telemetry.addData("currentStep", "turning back");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait a bit for momentum
        Thread.sleep(100);

        //reset back to 0 to be parallel
        drivetrain.moveForwardUntilZero(.5, 3500);

        //saftey stop
        drivetrain.stopMotors();

        //this ensures we are off the line
//        drivetrain.moveForward(.25, 100, 1000);

        //safety stop
//        drivetrain.stopMotors();

        //reset the encoders
        drivetrain.setNullValue();

        //move backwards until we touch the line
        drivetrain.moveFowardToLine(-.2, -.2, 4000);

        drivetrain.stopMotors();

        Thread.sleep(100);

        drivetrain.moveFowardToLine(.15, .15, 4000);

        //check the color and push the right color
        if (beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
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

//        drivetrain.moveForward(.25, 250);

        drivetrain.stopMotors();

        if(Math.abs(drivetrain.sensor.getGyroYaw()) > 2)
           drivetrain.rotatePZeroRev(.35);

        drivetrain.stopMotors();

        drivetrain.moveForward(.25, 100, 500);

        drivetrain.moveFowardToLine(.2, .21);  //This one corrects for drift but we are accurate with it

        drivetrain.stopMotors();

        Thread.sleep(100);

        drivetrain.moveFowardToLine(-.15, -.15, 3000); //move back to be aligned with white line

        drivetrain.stopMotors();

        Thread.sleep(100);

        telemetry.addData("currentStep", "finished");

        telemetry.addData("rightODS", drivetrain.sensor.rightODS());
        telemetry.addData("leftOdS", drivetrain.sensor.leftODS());
        telemetry.update();

        drivetrain.stopMotors();

        telemetry.addData("color", beaconPushers.getColorVal());
        telemetry.update();

        if (beaconPushers.isBackRed()){
            beaconPushers.backPush();
        }
        else {
            beaconPushers.frontPush();
        }
    }
}
