package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import android.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
<<<<<<< HEAD
<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
=======
import org.firstinspires.ftc.teamcode.Libraries.Lift;
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
import org.firstinspires.ftc.teamcode.Libraries.Lift;
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
import org.firstinspires.ftc.teamcode.Libraries.Lift;
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
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
<<<<<<< HEAD
<<<<<<< Updated upstream

    //create local variables
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD

    //create local variables
=======
=======
<<<<<<< HEAD
<<<<<<< HEAD

    //create local variables
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
=======
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
    private Lift lift;

    //create local variables

<<<<<<< HEAD
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
    private Lift lift;

    //create local variables

>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
    private double voltage;
    private String version;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize the robot
        drivetrain = new Drivetrain(this);
        manipulator = new Manipulator(this);
        shooter = new Shooter(this);
        beaconPushers = new BeaconPushers(this);
<<<<<<< HEAD
<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
=======
        lift = new Lift(this);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        lift = new Lift(this);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        lift = new Lift(this);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        //calculate the voltage
        voltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        /* This is the version number of the current iteration
        this is because sometimes the compiling process build the app but then installs
        the old version instead of applying updates. This version numbers is displayed over
        telemetry to ensure the autonomous is running the current version.
         */
        version = "1.133";

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
        Thread.sleep(500);

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

        //rotate 42 degrees to the left
<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.rotateP(.8, -32); /// 31 or so if going for first line
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.rotateP(.8, -32); /// 31 or so if going for first line
=======
        drivetrain.rotateP(.8, -33); /// 31 or so if going for first line
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.rotateP(.8, -33); /// 31 or so if going for first line
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.rotateP(.8, -33); /// 31 or so if going for first line
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        //stop after the rotation
        drivetrain.stopMotors();

        //show the current angle for testing purposes
        telemetry.addData("currentangle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait a bit for the momentum to stop moving the robot
        Thread.sleep(1000);

        //reset the encoder
        drivetrain.setNullValue();

        //display that we are going to move forward
        telemetry.addData("currentStep", "movingForward");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //move forward to the wall
<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveForwardToWall(.4, 4000);
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveForwardToWall(.4, 4000);
=======
=======
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveForwardToWall(.4, 4000);
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
        drivetrain.moveForwardToWall(.5, 4000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveForwardToWall(.5, 4000);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveForwardToWall(.5, 4000);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        //safety stop
        drivetrain.stopMotors();

        //display that we are going to even out with the wall
        telemetry.addData("currentStep", "turning back");
        telemetry.addData("currentAngle", drivetrain.sensor.getGyroYaw());
        telemetry.update();

        //wait a bit for momentum
        Thread.sleep(100);

        //reset back to 0 to be parallel

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.rotatePZero(.35);
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.rotatePZero(.35);
=======
        drivetrain.rotatePZero(.4);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.rotatePZero(.4);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.rotatePZero(.4);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        //saftey stop
        drivetrain.stopMotors();

        //this ensures we are off the line
//        drivetrain.moveForward(.25, 100, 1000);

        //safety stop
//        drivetrain.stopMotors();

        //reset the encoders
        drivetrain.setNullValue();

        //move backwards until we touch the line
<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveFowardToLine(.17, .35, 6000);
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(.17, .35, 6000);
=======
=======
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(.17, .35, 6000);
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
        drivetrain.moveFowardToLine(.15, .35, 6000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveFowardToLine(.15, .35, 6000);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveFowardToLine(.15, .35, 6000);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        drivetrain.stopMotors();

        Thread.sleep(100);

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveFowardToLine(-.12, -.17, 4000);
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(-.12, -.17, 4000);
=======
        drivetrain.moveFowardToLine(-.15, -.2, 4000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveFowardToLine(-.15, -.2, 4000);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveFowardToLine(-.15, -.2, 4000);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

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

        drivetrain.moveBackward(-.25, 100, 500);

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveFowardToLine(-.2, -.28);  //This one corrects for drift but we are accurate with it
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(-.2, -.28);  //This one corrects for drift but we are accurate with it
=======
=======
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(-.2, -.28);  //This one corrects for drift but we are accurate with it
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
        drivetrain.moveFowardToLine(-.15, -.35);  //This one corrects for drift but we are accurate with it
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveFowardToLine(-.15, -.35);  //This one corrects for drift but we are accurate with it
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveFowardToLine(-.15, -.35);  //This one corrects for drift but we are accurate with it
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        drivetrain.stopMotors();

        Thread.sleep(100);

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveFowardToLine(.12, .16, 3000); //move back to be aligned with white line
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveFowardToLine(.12, .16, 3000); //move back to be aligned with white line
=======
        drivetrain.moveFowardToLine(.13, .18, 3000); //move back to be aligned with white line
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveFowardToLine(.13, .18, 3000); //move back to be aligned with white line
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveFowardToLine(.13, .18, 3000); //move back to be aligned with white line
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

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

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveForward(.35, 750, 1000);
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveForward(.35, 750, 1000);
=======
=======
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveForward(.35, 750, 1000);
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
        drivetrain.moveForward(.6, 750, 1000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveForward(.6, 750, 1000);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveForward(.6, 750, 1000);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        while(drivetrain.sensor.getGyroYaw() < 95) {
            drivetrain.startMotors(.75, 0);
        }
        drivetrain.stopMotors();

<<<<<<< HEAD
<<<<<<< Updated upstream
        drivetrain.moveBackward(.75, 5500, 5000);
=======
<<<<<<< Updated upstream
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c
<<<<<<< HEAD
<<<<<<< HEAD
        drivetrain.moveBackward(.75, 5500, 5000);
=======
        drivetrain.moveBackward(1, 5500, 5000);
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
        drivetrain.moveBackward(1, 5500, 5000);
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
<<<<<<< HEAD
=======
        drivetrain.moveBackward(1, 5500, 5000);
>>>>>>> Stashed changes
>>>>>>> Stashed changes
=======
>>>>>>> 792ac9858ffc800a2104ba90be3a5003fe15265c

        drivetrain.stopMotors();
    }
}
