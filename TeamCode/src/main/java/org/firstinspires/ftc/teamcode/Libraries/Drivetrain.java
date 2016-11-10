package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    public Drivetrain(LinearOpMode opMode)throws InterruptedException {
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

    public void startMotors(double ri, double le) throws InterruptedException {
        motorR.setPower(ri);
        motorL.setPower(-le);
    }

    public void moveFowardToLine(double ri, double le) throws InterruptedException {
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        setNullValue();

        while(!sensor.isLeftLine()) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            opMode.telemetry.addData("LeftPower", motorL.getPower());
            opMode.telemetry.addData("RightPower", motorR.getPower());
            opMode.telemetry.update();

//            if(angle < startAngle - 2) {
//                startMotors((power * .75), power);
//            } else if(angle > startAngle + 2) {
//                startMotors(power, (power * .75));
//            } else {
//                startMotors(power, power);
//            }

            startMotors(ri, le);
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void stopMotors() throws InterruptedException {
        motorR.setPower(0);
        motorL.setPower(0);
    }

    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public int getEncoderAvg() {
        return ((Math.abs(motorR.getCurrentPosition())) + Math.abs(motorL.getCurrentPosition())) / 2;
    }

    public void moveForward(double pow, int encoderVal, double timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;
        setNullValue();

        opMode.resetStartTime();

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder || time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) + .2;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorL.getPower());
            opMode.telemetry.addData("RightPower", motorR.getPower());
            opMode.telemetry.addData("error", error);
            opMode.telemetry.update();

            if(angle < startAngle - 2) {
                startMotors((power * .75), power);
            } else if(angle > startAngle + 2) {
                startMotors(power, (power * .75));
            } else {
                startMotors(power, power);
            }
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void moveBackward(double pow, int encoderVal) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;
        setNullValue();

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        nullValue = 0;

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) - .25;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorL.getPower());
            opMode.telemetry.addData("RightPower", motorR.getPower());
            opMode.telemetry.update();

            if(angle < startAngle - 2) {
                startMotors((power), (power * .75));
            } else if(angle > startAngle + 2) {
                startMotors((power * .75), (power));
            } else {
                startMotors(power, power);
            }

            startMotors(power, power);
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void rotatePReset(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        sensor.resetGyro();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .008) + .1;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0015);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .025);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);
            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }


    public void rotateP(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .008) + .12;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .03);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZero(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .008) + .15;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .03);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            power *= -1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroRev(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .025) + .175;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .075);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power - inteNoE - der;

            power *= 1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroB(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .025) + .175;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .075);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            power *= -1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroRevB(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .008) + .1;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .03);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power - inteNoE - der;

            power *= 1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePB(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .0025) + .1;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .05);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        stopMotors();
        opMode.telemetry.addData("finished", "done");
        opMode.telemetry.update();
    }
}

