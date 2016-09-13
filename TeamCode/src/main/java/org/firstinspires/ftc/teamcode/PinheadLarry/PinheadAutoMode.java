package org.firstinspires.ftc.teamcode.PinheadLarry;

import android.media.MediaPlayer;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.AdafruitIMU;
import org.firstinspires.ftc.teamcode.Libraries.MotorScaler;

/**
 * Created by Arib on 4/19/2016.
 */
//blank lines are replaced
public abstract class PinheadAutoMode extends LinearOpMode {

    //====================BEGIN CREATING OBJECTS====================
    //Adafruit BNO055 gyro
    public AdafruitIMU gyro;

    //motors
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor liftL;
    public DcMotor liftR;
    public DcMotor mani;

    //servos
    public Servo conveyer;
    public Servo rightDoor;
    public Servo leftDoor;
    public Servo rightPaddle;
    public Servo leftPaddle;
    public Servo latch;

    //DeviceInterfaveModule for sensors
    public DeviceInterfaceModule cdim;

    //MediaPlayer object to get the RobotController to play sounds
    public MediaPlayer song;
    //====================END CREATING OBJECTS====================

    //====================BEGIN CREATING MATH VARIABLES====================
    //coordinate system variables
    private int xTile;
    private int yTile;
    int facing;     //SPECIAL NOTE: It made much more sense to use integers as a substitute for chars
    // while coding the heading. I used a compass type orientation for movement in
    // the correct direction. 1 = North, 2 = East, 3 = South, 4 = West

    int nullValue;
    public double angleError;
    int avgEncoderDistance;
    private static final double WHEEL_DIAMETER = 4;
    private static final double DISTANCE_PER_ROTATION = WHEEL_DIAMETER * Math.PI;
    private static final int SINGLE_ROTATION = 1120;

    //volatile double array to hold gyro angle
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    //final variables to hold servo positions
    private final double RIGHT_DOOR_UP = 1;
    private final double RIGHT_DOOR_DOWN = .5;
    private final double LEFT_DOOR_UP = .35;
    private final double LEFT_DOOR_DOWN = .85;
    private final double RIGHT_PADDLE_OUT = 1;
    private final double RIGHT_PADDLE_IN = .5;
    private final double LEFT_PADDLE_OUT = 0;
    private final double LEFT_PADDLE_IN = .5;
    private final double LATCH_DOWN = .5;
    private final double LATCH_UP = .5;
    //====================END CREATING MATH VARIABLES====================

    //====================BEGIN CONSTRUCTORS AND INITIALIZER====================
    //constructor
    public PinheadAutoMode() {
        super(); //ADDED ON 3/1/2016
    }

    //Hardware map and initialize motors and servos
    public final void first() {
        cordFirst(6, 10, 1);

    }

    public final void cordFirst(int xTile, int yTile, int facing) {
        this.xTile = xTile;
        this.yTile = yTile;
        this.facing = facing;
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        liftL = hardwareMap.dcMotor.get("liftl");
        liftR = hardwareMap.dcMotor.get("liftr");
        mani = hardwareMap.dcMotor.get("mani");
        conveyer = hardwareMap.servo.get("conv");
        rightDoor = hardwareMap.servo.get("rd");
        leftDoor = hardwareMap.servo.get("ld");
        rightPaddle = hardwareMap.servo.get("rp");
        leftPaddle = hardwareMap.servo.get("lp");
        latch = hardwareMap.servo.get("latch");
//        song = MediaPlayer.create(FtcRobotControllerActivity.getContext(), R.raw.move); //TODO: UPDATE RAW FILE WITH FILES
        song.setLooping(true);
        song.seekTo(5000);
        song.start();
        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
            telemetry.addData("gyro", "fail");
        }

        hardwareMap.logDevices();
        nullValue = 0;

        rightPaddleIn();
        leftPaddleIn();
        closeRightDoor();
        closeLeftDoor();
        stopConv();

        latch.setPosition(.5);
        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");
        telemetry.addData("Auto", "Initialized Successfully!");
    }
    //====================END CONSTRUCTORS AND INITIALIZER====================

    //====================BEGIN MOVEMENT METHODS====================
    //This method moves forward at a given power to a given encoder value
    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        //notifies Driver Station of current event
        telemetry.addData("auto", "Moving Forwards");

        //resets the Gyro's angle
        resetGyro();
        double angle;
        sendData();

        setNullValue();

        int currentEncoder = getBackWheelAvg() - nullValue;
        //while target is not reached
        while((encoderVal > currentEncoder)) {
            telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
            telemetry.addData("avg", getBackWheelAvg());
            sendData();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            angle = yawAngle[0];

            currentEncoder = getBackWheelAvg() - nullValue;

            //if off to the left, correct
            if(angle > 2) {
                startMotors(pow, (pow * .75));
                sendData();
            } else if(angle < -2) { //if off to the right, correct
                startMotors((pow * .75), pow);
                sendData();
            } else { //if heading is fine keep moving straight
                startMotors(pow, pow);
                sendData();
            }
            idle();
        }

        //once finished stop moving and send data
        stopMotors();
        sendData();
        angleError = yawAngle[0];
    }

    public void moveForwardPID(double pow, int encoderVal) throws InterruptedException {
        //notifies Driver Station of current event
        telemetry.addData("auto", "Moving Forwards");

        //resets the Gyro's angle
        resetGyro();
        double angle;
        sendData();

        double error = 0;
        double power = pow;
        setNullValue();

        int currentEncoder = getBackWheelAvg() - nullValue;
        //while target is not reached
        while(encoderVal > currentEncoder) {
            telemetry.addData("BL", Math.abs(motorBL.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(motorBR.getCurrentPosition()));
            telemetry.addData("avg", getBackWheelAvg());
            sendData();
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            angle = yawAngle[0];

            currentEncoder = getBackWheelAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) + .25;

            if(power > 1) {
                power = 1;
            }

            if(power < -1) {
                power = -1;
            }

            telemetry.addData("Power", power);


            telemetry.addData("LeftPower", motorBL.getPower() + "");
            telemetry.addData("RightPower", motorBR.getPower() + "");

            //if off to the left, correct
            if(angle > 2) {
                startMotors((power * .75), power);
                sendData();
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");
            } else if(angle < -2) { //if off to the right, correct
                startMotors(power, (power * .75) );
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");
                sendData();
            } else { //if heading is fine keep moving straight
                startMotors(power, power);
                sendData();
                telemetry.addData("LeftPower", motorBL.getPower() + "");
                telemetry.addData("RightPower", motorBR.getPower() + "");

            }

        }

        //once finished stop moving and send data

        stopMotors();
        sendData();
        angleError = yawAngle[0];

    }

    public void moveForwardScaled(double pow, int encoderVal) throws InterruptedException {
        pow = MotorScaler.scaleSimple(pow);
        moveForward(pow, encoderVal);
    }

    //this method calls child methods to make commands easier
    public void moveToCoordinatePos(int xTileTo, int yTileTo) throws InterruptedException {
        int yDiff = yTile - yTileTo;    //calculates differences between current position and goal position
        int xDiff = xTile - xTileTo;
        if(yDiff < 0 && facing != 3)                   //set the heading depending on goal *see note for facing var*
            setFacing(3);

        if(yDiff > 0 && facing != 1)
            setFacing(1);

        moveXTiles(Math.abs(yDiff));    //uses a method to move the difference in tiles forward
        pRotateNoReset(-.2, 0);

        if(xDiff < 0 && facing != 4)                   //resets the heading depending on goal for other axis
            setFacing(4);

        if(xDiff > 0 && facing != 2)
            setFacing(2);

        moveXTiles(Math.abs(xDiff));    //uses a method to move the difference in tiles forward
        pRotateNoReset(-.2, 0);         //this corrects for any drift. Only corrects yaw. does not calculate displacement. yet.

        stopMotors();                   //stop the motors in case something goes wrong somewhere else

    }

    //TODO: FIX THIS METHOD SO THAT IT ACTUALLY WORKS
    public void moveToCordinatePosAngle(int xTileTo, int yTileTo) throws InterruptedException {
        int yDiff = yTile - yTileTo;
        int xDiff = xTile - xTileTo;
        boolean firstTurn = false;
        if(yDiff < 0 && facing != 1 && !firstTurn)                   //set the heading depending on goal *see note for facing var*
            setFacing(3);

        if(yDiff > 0 && facing != 1 && !firstTurn)
            setFacing(1);


        //All of this math can probably be done in a better manner
        //TODO: FIX THIS MATH TO MAKE IT EASIER TO READ AND FOLLOW THIS MATH IS ALSO PROBABLY WRONG PRETTY MUCH JUST REDO THIS.
        int absyDiff = Math.abs(yDiff);                             //create absolute values for calculations
        int absxDiff = Math.abs(xDiff);
        double angleToTurn = Math.sin(absxDiff / absyDiff);         //calculate the angle to turn using trigonometry
        pRotate(.2, angleToTurn);                                   //send the command to turn

        double oneTileInInches = 24;                                                //encoder value calculation for one tile
        Double xdistToMoveInches = oneTileInInches * xDiff;                          //encoder value calculation for multiple tiles
        double xRotationsToMove = xdistToMoveInches / DISTANCE_PER_ROTATION;          //cast the encoder value as an integer
        double xencoderTicksToMoveSquared = Math.pow(Math.round(xRotationsToMove * SINGLE_ROTATION), 2);

                                                                                     //calculate the amount of ticks for the other movement
        Double ydistToMoveInches = oneTileInInches * yDiff;                          //encoder value calculation for multiple tiles
        double yRotationsToMove = ydistToMoveInches / DISTANCE_PER_ROTATION;          //cast the encoder value as an integer
        double yencoderTicksToMoveSquared = Math.pow(Math.round(yRotationsToMove * SINGLE_ROTATION), 2);

        int hypotenuseTicks = (int) Math.sqrt(xencoderTicksToMoveSquared + yencoderTicksToMoveSquared); //use pythagorean theorem to calculate the
        moveForwardPID(.5, hypotenuseTicks);

        pRotateNoReset(-.2, 0);                                                     //straighten out if any drift occurs

    }

    //changes the heading
    public void setFacing(int f) throws InterruptedException {
        int diff = f - facing;      //calculates the difference between current heading and goal
        double angle = 90;         //changes the difference to an angle value
        double pow = .2;            //default power of half
        if(angle > 0)               //set the power to turn the correct way
            pow = .2;
        if(angle < 0)
            pow = -.2;
        if(diff != 0)               //if change is not needed do not move
            pRotate(pow, angle);    //uses a PID loop for accurate rotation
    }

    //moves forward a certain number of tiles
    public void moveXTiles(int numTiles) throws InterruptedException {
        double oneTileInInches = 24;                                                //encoder value calculation for one tile
        Double distToMoveInches = oneTileInInches * numTiles;                       //encoder value calculation for multiple tiles
        double rotationsToMove = distToMoveInches / DISTANCE_PER_ROTATION;          //cast the encoder value as an integer
        int encoderTicksToMove = (int)Math.round(rotationsToMove * SINGLE_ROTATION);
        moveForwardPID(.5, encoderTicksToMove);                                        //move forward given tiles
        stopMotors();
    }

    //rotate the robot
    public void rotate() throws InterruptedException {

        resetGyro();
        sendData();
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double angle = yawAngle[0];
        //while current angle is greater than desired angle
        while(angle > -15) {
            sendData();

            startMotors(.2, .2);

            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

            angle = yawAngle[0];
        }
        stopMotors();
        //if overshot
        while(angle < -20) {

            startMotors(-.2, -.2);
            sendData();

            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

            angle = yawAngle[0];
        }
        stopMotors();
        //if undershot
        while(angle > -10) {

            startMotors(.2, .2);
            sendData();

            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

            angle = yawAngle[0];
        }
        stopMotors();
        sendData();

    }

    //P loop for PID //TODO: UPDATE VALS AND TEST
    public void pRotate(double pow, double angle) throws InterruptedException {
        resetGyro();
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        double error;
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double currentAngle = yawAngle[0];
        double previousError = angleTo - currentAngle;
        //telemetry data for which step we are currently on
        telemetry.addData("auto", "rotate");
        //do while
        while (Math.abs(currentAngle) < angleTo - 2) {
            getAngles();                                //update angles
            currentAngle = yawAngle[0];                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if(power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }
            startMotors(-power, power);                 //set the motors to turn
            getAngles();
            telemetry.addData("Gyro", yawAngle[0]);     //send data to Driver station
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("AngleTo", angleTo);
            getAngles();
            telemetry.addData("currentAngle", currentAngle);

            sendData();
            telemetry.addData("PID", power);
            previousError = error;
            getAngles();
        }
        sendData();
        stopMotors();                                  //stop motion
        Double d = angle;
        int rotated = d.intValue();
        Double ticks = rotated / 90.0;
        int ticksI = ticks.intValue();
        updateFacing(ticksI);
    }

    public void pRotateNoReset(double pow, double angle) throws InterruptedException {
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        double currentAngle = yawAngle[0];
        double previousError = angleTo - currentAngle;
        double error = angleTo - currentAngle;
        //telemetry data for which step we are currently on
        telemetry.addData("auto", "rotate");
        //while
        while (currentAngle < angleTo) {
            getAngles();                                //update angles
            currentAngle = yawAngle[0];                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .0005) + .145;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if(power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }
            startMotors(power, -power);                 //set the motors to turn
            getAngles();
            telemetry.addData("Gyro", yawAngle[0]);     //send data to Driver station
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("AngleTo", angleTo);
            getAngles();
            telemetry.addData("currentAngle", currentAngle);

            sendData();
            telemetry.addData("PID", power);
            previousError = error;
            getAngles();
        }
        sendData();
        stopMotors();                                  //stop motion
        Double d = angle;
        int rotated = d.intValue();
        Double ticks = rotated / 90.0;
        int ticksI = ticks.intValue();
        updateFacing(ticksI);
    }
    //====================END MOVEMENT METHODS====================

    //====================BEGIN COMMANDS====================


    //start the motors in a tank drive
    public void startMotors(double ri, double le) throws InterruptedException {
        motorBL.setPower(-le);

        motorBR.setPower(ri);

        motorFL.setPower(-le);

        motorFR.setPower(ri);

    }

    //start the motors in a tank drive
    public void startMotorsScaled(double ri, double le) throws InterruptedException {
        ri = MotorScaler.scaleSimple(ri);
        le = MotorScaler.scaleSimple(le);
        motorBL.setPower(le);

        motorBR.setPower(-ri);

        motorFL.setPower(le);

        motorFR.setPower(-ri);

    }

    //stop all the motors
    public void stopMotors() throws InterruptedException {

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);

    }

    public void rightPaddleOut() {
        rightPaddle.setPosition(RIGHT_PADDLE_OUT);
    }

    public void extendLifts(double pow) {
        liftL.setPower(pow);
        liftR.setPower(-pow);
    }

    public void stopLifts() {
        liftL.setPower(0);
        liftR.setPower(0);
    }

    public void startMani() {
        mani.setPower(1);
    }

    public void reverseMani() {
        mani.setPower(-1);
    }

    public void stopMani() {
        mani.setPower(0);
    }

    public void openRightDoor() {
        rightDoor.setPosition(RIGHT_DOOR_DOWN);
    }

    public void openLeftDoor() {
        leftDoor.setPosition(LEFT_DOOR_DOWN);
    }

    public void closeRightDoor() {
        rightDoor.setPosition(RIGHT_DOOR_UP);
    }

    public void closeLeftDoor() {
        leftDoor.setPosition(LEFT_DOOR_UP);
    }

    public void moveConvRight() {
        conveyer.setPosition(1);
    }

    public void moveConvLeft() {
        conveyer.setPosition(0);
    }

    public void stopConv() {
        conveyer.setPosition(.5);
    }

    public void rightPaddleIn() {
        rightPaddle.setPosition(RIGHT_PADDLE_IN);
    }

    public void leftPaddleOut() {
        leftPaddle.setPosition(LEFT_PADDLE_OUT);
    }

    public void leftPaddleIn() {
        leftPaddle.setPosition(LEFT_PADDLE_IN);
    }

    public void latchDown() {
        latch.setPosition(LATCH_DOWN);
    }


    //====================END COMMANDS====================

    //====================BEGIN CALCULATION METHODS====================

    private void updateCoordinates() {
        Double inchesTraveled = (avgEncoderDistance / SINGLE_ROTATION) * WHEEL_DIAMETER;
        int inchesTraveledI = inchesTraveled.intValue();
        if(facing == 1) {
            yTile = yTile - inchesTraveledI;
        }
        if(facing == 2) {
            yTile = yTile + inchesTraveledI;
        }
        if(facing == 3) {
            xTile = xTile + inchesTraveledI;
        }
        if(facing == 4) {
            xTile = xTile - inchesTraveledI;
        }
    }

    private void updateFacing(int ticks) {
        for(int i = 0; i < ticks; i++) {
            facing++;
        }
        facing = facing / 4;
    }

    //reset the gyro and its angles
    public void resetGyro() throws InterruptedException {
        gyro.startIMU();

    }

    public void setNullValue() throws InterruptedException {
        nullValue = getBackWheelAvg();
    }

    //get the encoder average of all the wheels
    public int getEncoderAvg() {
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition()))
                + (Math.abs(motorFL.getCurrentPosition())) + (Math.abs(motorFR.getCurrentPosition())))
                / 4;
    }

    //get the encoder average of the back wheels
    public int getBackWheelAvg() {
        avgEncoderDistance = ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())))
                / 2;
        return ((Math.abs(motorBL.getCurrentPosition())) + (Math.abs(motorBR.getCurrentPosition())))
                / 2;

    }

    public void getAngles() throws InterruptedException {
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

    }

    //check to be sure if pitch is ok
    public boolean isOk() throws InterruptedException {

        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        double angle = pitchAngle[0];
        if(angle > .5) {
            return false;
        }
        return true;
    }

    //check to see if heading and pitch are ok after rotation
    public boolean allIsOk() throws InterruptedException {

        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        double rotate = yawAngle[0];
        double pitch = pitchAngle[0];
        if((rotate < 17 && rotate > 13) && pitch < 2.5) {
            return true;
        }
        return false;
    }

    //get and telemetry data to Driver Station
    public void sendData() throws InterruptedException {
        getAngles();
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
        telemetry.addData("Max I2C read interval: ",
                String.format("%4.4f ms. Average interval: %4.4f ms.", gyro.maxReadInterval
                        , gyro.avgReadInterval));


    }

    //====================END CALCULATION METHODS====================
}
