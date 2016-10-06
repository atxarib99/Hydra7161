package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Arib on 10/6/2016.
 */
public class Drivetrain {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    LinearOpMode opMode;
    public Drivetrain(LinearOpMode opMode){
        this.opMode = opMode;
        FR = this.opMode.hardwareMap.dcMotor.get("FR");
        BL = this.opMode.hardwareMap.dcMotor.get("BL");
        FL = this.opMode.hardwareMap.dcMotor.get("FL");
        BR = this.opMode.hardwareMap.dcMotor.get("BR");
        this.opMode.telemetry.addData("init", "init finished");
        this.opMode.telemetry.update();
    }

    public void startMotors(double ri, double le) {
        FR.setPower(ri);
        FL.setPower(-le);
        BR.setPower(ri);
        BL.setPower(-le);
    }

    public void stopMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void moveForward(double power, int encoderVal) {

    }

//    public void moveForwardPID(double power, int encoderVal) {
//
//            resetGyro();
//            double angle;
//            telemetry.update();
//
//            double error;
//            double power;
//            setNullValue();
//
//            int currentEncoder = getEncoderAvg() - nullValue;
//            while(encoderVal > currentEncoder) {
//                telemetry.update();
//                getAngles();
//                angle = getGyroYaw();
//
//                currentEncoder = getEncoderAvg() - nullValue;
//
//                error = (double) (encoderVal - currentEncoder) / encoderVal;
//
//                power = (pow * error) + .25;
//
//                Range.clip(power, -1, 1);
//
//                telemetry.addData("Power", power);
//                telemetry.addData("LeftPower", motorBL.getPower());
//                telemetry.addData("RightPower", motorBR.getPower());
//
//                if(angle > 2) {
//                    startMotors((power * .75), power);
//                    telemetry.update();
//                    telemetry.addData("LeftPower", motorBL.getPower() + "");
//                    telemetry.addData("RightPower", motorBR.getPower() + "");
//                } else if(angle < -2) { //if off to the right, correct
//                    startMotors(power, (power * .75) );
//                    telemetry.addData("LeftPower", motorBL.getPower() + "");
//                    telemetry.addData("RightPower", motorBR.getPower() + "");
//                    telemetry.update();
//                } else { //if heading is fine keep moving straight
//                    startMotors(power, power);
//                    telemetry.update();
//                    telemetry.addData("LeftPower", motorBL.getPower() + "");
//                    telemetry.addData("RightPower", motorBR.getPower() + "");
//
//                }
//                idle();
//            }
//            stopMotors();
//            telemetry.update();
//            angleError = getGyroYaw();
//        }
//    }

    public void rotateP(int deg) {

    }
}
