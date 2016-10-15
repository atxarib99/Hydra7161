package org.firstinspires.ftc.teamcode.PinheadLarry;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OldFiles.AdafruitIMU;

/**
 * Created by Arib on 4/11/2016.
 */
public abstract class PinheadOpMode extends OpMode {

    public AdafruitIMU gyro;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor liftL;
    public DcMotor liftR;
    public DcMotor mani;
    public Servo conveyer;
    public Servo rightDoor;
    public Servo leftDoor;
    public Servo rightPaddle;
    public Servo leftPaddle;
    public Servo latch;
    public DeviceInterfaceModule cdim;

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    private final double RIGHT_DOOR_UP = 1;
    private final double RIGHT_DOOR_DOWN = .35;
    private final double LEFT_DOOR_UP = .35;
    private final double LEFT_DOOR_DOWN = .85;
    private final double RIGHT_PADDLE_OUT = 1;
    private final double RIGHT_PADDLE_IN = .5;
    private final double LEFT_PADDLE_OUT = 0;
    private final double LEFT_PADDLE_IN = .5;
    private final double LATCH_DOWN = .1;
    private final double LATCH_UP = .6;

    @Override
    public void init() {
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
        telemetry.addData("gyro", "initializing...");

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

        telemetry.addData("gyro", gyro == null? "BAD":"GOOD");
        telemetry.addData("init", "pass");

        rightPaddleIn();
        leftPaddleIn();
        closeRightDoor();
        closeLeftDoor();
        stopConv();
        latch.setPosition(.5);
        
    }

    @Override
    public void start() {
        super.start();
        gyro.startIMU();
    }


    @Override
    public void stop() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
    }

    public void startMotors(double ri, double le) {
        motorBL.setPower(le);
        motorBR.setPower(-ri);
        motorFL.setPower(le);
        motorFR.setPower(-ri);
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
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

    public void getAngles() {
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("yaw", yawAngle[0]);
    }

    public void rightPaddleOut() {
        rightPaddle.setPosition(RIGHT_PADDLE_OUT);
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

    public void latchUp() {
        latch.setPosition(LATCH_UP);
    }





}
