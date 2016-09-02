package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Created by Owner on 8/31/2015.
 */
public class IMUGyroOnlyTest extends PinheadOpMode {


    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    private static final double UNDROPPED = 0;
    private static final double DROPPED = 1;
    private static final double RIGHTPADDLE_OUT = 0;
    private static final double LEFTPADDLE_OUT = 1;
    private static final double RIGHTPADDLE_IN = 1;
    private static final double LEFTPADDLE_IN = 0;
    private static final double BASKET_DUMPED = .9;
    private static final double BASKET_IDLE = .5;
    private static final double BASKET_LEFT = 1;
    private static final double BASKET_RIGHT = 0;

    long systemTime;//Relevant values of System.nanoTime

    /************************************************************************************************
     * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
     * before "start" runs.
     */
    //can be foudn in PinheadOpMode

    /************************************************************************************************
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void start() {
        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
        systemTime = System.nanoTime();
        gyro.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
        Log.i("FtcRobotController", "IMU Start method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    }

    /***********************************************************************************************
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
     * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
     * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
     * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
     * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
     * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
     */
    @Override
    public void loop() {
        //Log.i("FtcRobotController", "Loop method starting at: " +
        //      -(systemTime - (systemTime = System.nanoTime())) + " since last loop start.");

        // write the values computed by the "worker" threads to the motors (if any)

        //Read the encoder values that the "worker" threads will use in their computations
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
		/*
		 * Send whatever telemetry data you want back to driver station.
		 */
        //telemetry.addData("Text", "*** Robot Data***");

        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            motorBL.setPower(gamepad1.left_stick_y);
            motorBR.setPower(-gamepad1.right_stick_y);
            motorFR.setPower(-gamepad1.right_stick_y);
            motorFL.setPower(gamepad1.left_stick_y);
        } else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }

        telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
        telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
        telemetry.addData("avg", (((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))) / 2));
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
        telemetry.addData("Max I2C read interval: ",
                String.format("%4.4f ms. Average interval: %4.4f ms.", gyro.maxReadInterval
                        , gyro.avgReadInterval));
    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {
        //When the FTC Driver Station's "Start with Timer" button commands autonomous mode to start,
        //then stop after 30 seconds, stop the motors immediately!
        //Following this method, the underlying FTC system will call a "stop" routine of its own
        systemTime = System.nanoTime();
        Log.i("FtcRobotController", "IMU Stop method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    }
}