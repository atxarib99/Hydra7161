package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;
import java.util.regex.Matcher;

/**
 * Created by Arib on 10/6/2016.
 */
public class Drivetrain {
    DcMotor motorR;
    DcMotor motorL;

    LinearOpMode opMode;

    public Sensor sensor;

    int nullValue;
    double angleError;

    private final String LOG_TAG = "DriveTrain";
    public Drivetrain(LinearOpMode opMode){
        this.opMode = opMode;
        nullValue = 0;
        motorL = this.opMode.hardwareMap.dcMotor.get("L");
        motorR = this.opMode.hardwareMap.dcMotor.get("R");
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
        sensor = new Sensor(opMode);
        this.opMode.telemetry.addData(LOG_TAG + "init", "init finished");
        this.opMode.telemetry.update();
    }

    public void startMotors(double ri, double le) {
        motorR.setPower(ri);
        motorL.setPower(-le);
    }

    public void stopMotors() {
        motorR.setPower(0);
        motorL.setPower(0);
    }

    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public int getEncoderAvg() {
        return  ((Math.abs(motorR.getCurrentPosition())) + Math.abs(motorL.getCurrentPosition())) / 2;
    }

    public void moveForward(double pow, int encoderVal) throws InterruptedException {
        sensor.resetGyro();
        double angle;
        opMode.telemetry.update();

        double error;
        double power;
        setNullValue();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder) {
            opMode.telemetry.update();
            angle = sensor.getGyroYaw();

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) + .25;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorL.getPower());
            opMode.telemetry.addData("RightPower", motorR.getPower());
            opMode.telemetry.update();

            if(angle > 2) {
                startMotors((power * .75), power);
                opMode.telemetry.addData("LeftPower", motorL.getPower());
                opMode.telemetry.addData("RightPower", motorR.getPower());
                opMode.telemetry.update();
            } else if(angle < -2) {
                startMotors(power, (power * .75));
                opMode.telemetry.addData("LeftPower", motorL.getPower());
                opMode.telemetry.addData("RightPower", motorR.getPower());
                opMode.telemetry.update();
            } else {
                startMotors(power, power);
                opMode.telemetry.addData("LeftPower", motorL.getPower());
                opMode.telemetry.addData("RightPower", motorR.getPower());
                opMode.telemetry.update();
            }
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void rotateP(double pow, int deg) throws InterruptedException {
        sensor.resetGyro();

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        while(Math.abs(currentAngle) < angleTo - 2) {
            currentAngle = sensor.getGyroYaw();
            error = angleTo - Math.abs(currentAngle);                   //TODO: UPDATE PID VALS *higher vals since less motors and 20s*
            power = (pow * (error) * .005);                      //update p values
            inte += (opMode.getRuntime() * error * .005);         //update inte value
            der = (error - previousError) / opMode.getRuntime() * .005; //update der value

            power += inte + der;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }
}
