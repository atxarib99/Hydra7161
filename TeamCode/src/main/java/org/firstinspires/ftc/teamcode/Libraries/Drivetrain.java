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
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorFL;

    LinearOpMode opMode;

    public Sensor sensor;

    int nullValue;
    double angleError;

    public boolean reversed;

    private final String LOG_TAG = "DriveTrain";
    public Drivetrain(LinearOpMode opMode)throws InterruptedException {
        this.opMode = opMode;
        nullValue = 0;
        motorBL = this.opMode.hardwareMap.dcMotor.get("BL");
        motorBR = this.opMode.hardwareMap.dcMotor.get("BR");
        motorFL = this.opMode.hardwareMap.dcMotor.get("FL");
        motorFR = this.opMode.hardwareMap.dcMotor.get("FR");
        reversed = false;
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
        sensor = new Sensor(opMode);
        this.opMode.telemetry.addData(LOG_TAG + "init", "init finished");
        this.opMode.telemetry.update();
    }

    public void startMotors(double ri, double le) throws InterruptedException {
        if(reversed) {
            motorBL.setPower(-le);
            motorFL.setPower(-le);
            motorBR.setPower(ri);
            motorFR.setPower(ri);
        } else {
            motorBL.setPower(ri);
            motorFL.setPower(ri);
            motorBR.setPower(-le);
            motorFR.setPower(-le);
        }
    }

    public void moveFowardToLine(double ri, double le) throws InterruptedException {
        moveFowardToLine(ri, le, 10000);
    }

    public void moveFowardToLine(double ri, double le, int timeout) throws InterruptedException {
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        setNullValue();
        ElapsedTime time = new ElapsedTime();
        time.startTime();

        while(!sensor.isLeftLine() && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());

            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
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

    public void moveForwardToWall(double pow, int timeout) throws InterruptedException {
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angle;

        double power = pow;

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while (Math.abs(angle - startAngle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if (angle < startAngle - 1) {
                startMotors((power * .75), power);
            } else if (angle > startAngle + 1) {
                startMotors(power, (power * .75));
            } else {
                startMotors(power, power);
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }

    public void moveBackwardToWall(double pow, int timeout) throws InterruptedException {
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angle;

        double power = pow;

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while (Math.abs(angle - startAngle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if (angle < startAngle - 1) {
                startMotors((power), (power * .75));
            } else if (angle > startAngle + 1) {
                startMotors((power*.75), (power));
            } else {
                startMotors(power, power);
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }


    public void moveForwardToWallEnc(double pow, int timeout) throws InterruptedException {
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angle;

        double power = pow;

        int tick = 1;

        resetEncoders();

        int startEncoder = Math.abs(motorFL.getCurrentPosition());

        int endEncoder;

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while (Math.abs(angle - startAngle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();


            if(time.milliseconds() > 250 * tick) {
                endEncoder = Math.abs(motorFL.getCurrentPosition());
                if(startEncoder + 250 < endEncoder) {
                    break;
                }
            }

            if (angle < startAngle - 1) {
                startMotors((power * .75), power);
            } else if (angle > startAngle + 1) {
                startMotors(power, (power * .75));
            } else {
                startMotors(power, power);
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }

    public void moveForwardUntilZero(double pow, double timeout) throws InterruptedException {

        double angle = sensor.getGyroYaw();

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while(Math.abs(angle) > 2 && time.milliseconds() < timeout) {
            startMotors(-.3, -.5);

            opMode.telemetry.addData("current", "zeroout");
            opMode.telemetry.update();

            opMode.idle();
        }

        stopMotors();
    }

    public void stopMotors() throws InterruptedException {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public void reverse() {
        reversed = !reversed;
    }

    public int getEncoderAvg() {
        return ((Math.abs(motorBR.getCurrentPosition())) + (Math.abs(motorBL.getCurrentPosition())) +
                (Math.abs(motorFR.getCurrentPosition())) + (Math.abs(motorFL.getCurrentPosition()))) / 4;
    }

    public void resetEncoders() throws InterruptedException {

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

    }

    public void moveForward(double pow, int encoderVal, double timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        setNullValue();

        opMode.resetStartTime();

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            if(pow < 0)
                power = (pow * error) - .2;
            else
                power = (pow * error) + .2;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.addData("error", error);
            opMode.telemetry.update();

//            if(angle > startAngle + 2) {
//                startMotors((power * .75), power);
//            } else if(angle < startAngle - 2) {
//                startMotors(power, (power * .75));
//            } else {
//                startMotors(power, power);
//            }
            startMotors(power, power);
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void moveBackward(double pow, int encoderVal, int timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();


        setNullValue();

        nullValue = 0;

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            power = (pow * error) - .25;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
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
<<<<<<< Updated upstream
            power = (pow * (error) * .007) + .12;                      //update p values
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
            power = (pow * (error) * .007) + .12;                      //update p values
=======
            power = (pow * (error) * .01) + .12;                      //update p values
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
            power = (pow * (error) * .01) + .12;                      //update p values
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
=======
            power = (pow * (error) * .01) + .12;                      //update p values
>>>>>>> Stashed changes
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> Stashed changes
        while(Math.abs(currentAngle) > 6) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .0075) + .13;                      //update p values
<<<<<<< Updated upstream
=======
=======
=======
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
        while(Math.abs(currentAngle) > 10) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .015) + .13;                      //update p values
<<<<<<< HEAD
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
=======
        while(Math.abs(currentAngle) > 10) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .015) + .13;                      //update p values
>>>>>>> Stashed changes
>>>>>>> Stashed changes
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
            power = (pow * (error) * .02) + .125;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .075);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE - der;

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
        double angleTo = -180;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;
        double error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(error > 10) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
<<<<<<< Updated upstream
            power = (pow * (error) * .005) + .1;                      //update p values
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
            power = (pow * (error) * .005) + .1;                      //update p values
=======
            power = (pow * (error) * .01) + .1;                      //update p values
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
            power = (pow * (error) * .01) + .1;                      //update p values
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
=======
            power = (pow * (error) * .01) + .1;                      //update p values
>>>>>>> Stashed changes
>>>>>>> Stashed changes
            inte = ((opMode.getRuntime()) * error * .0015);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .05);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

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
<<<<<<< Updated upstream
            power = (pow * (error) * .0025) + .1;                   //update p values
=======
<<<<<<< Updated upstream
<<<<<<< HEAD
<<<<<<< HEAD
            power = (pow * (error) * .0025) + .1;                   //update p values
=======
            power = (pow * (error) * .005) + .1;                   //update p values
>>>>>>> Added all the lift functionality to the Autonomous. Edited values of autonomous to compensate for the lifts weight. Values still in testing.
=======
            power = (pow * (error) * .005) + .1;                   //update p values
>>>>>>> b9bc9ea238cbf8e12a36b483e13136c2771ad3ef
=======
            power = (pow * (error) * .005) + .1;                   //update p values
>>>>>>> Stashed changes
>>>>>>> Stashed changes
            inte = ((opMode.getRuntime()) * error * .005);          //update inte value
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

