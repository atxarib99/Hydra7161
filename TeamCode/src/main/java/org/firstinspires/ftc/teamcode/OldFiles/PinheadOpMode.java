package org.firstinspires.ftc.teamcode.OldFiles;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 4/11/2016.
 */
public abstract class PinheadOpMode extends OpMode {

    public AdafruitIMU gyro;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor motorFR;
    public DcMotor motorFL;

    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        telemetry.addData("gyro", "initializing...");

        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
            try {
                Thread.sleep(5000);
            } catch(InterruptedException e) {
                Log.i("PinheadOpMode", e.getMessage());
            }

            gyro.startIMU();
        } catch (RobotCoreException e){
            Log.i("PinheadOpMode", e.getMessage());
            telemetry.addData("gyro", "fail");
        }
        telemetry.addData("init", "pass");
        
    }


    @Override
    public void stop() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
    }

}
