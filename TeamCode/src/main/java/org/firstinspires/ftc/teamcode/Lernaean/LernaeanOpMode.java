package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by Arib on 9/11/2016.
 */
public abstract class LernaeanOpMode extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor manipulator;
    DcMotor shooterR;
    DcMotor shooterL;
    DcMotor liftRelease;
    Servo front;
    Servo back;
    Servo activate;
    Servo armRight;
    Servo armLeft;
    Servo armRelease;
    Servo topGrabber;
    DeviceInterfaceModule cdim;
    ColorSensor color;
    ColorSensor color2;
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;


    private final double BACK_OUT = 0;
    private final double BACK_IN = 1;
    private final double FRONT_OUT = 0;
    private final double FRONT_IN = 1;
    private final double ARM_IN = 1;
    private final double ARM_GRAB = .9;
    private final double ARM_OPEN = .525;
    private final double ARM_DROP = .25;
    private final double ARM_CLOSE = 0;
    private final double LIFT_UNACTIVATED = 0;
    private final double LIFT_ACTIVATED = 1;
    private final double ARM_RELEASER_RELEASED = 1;
    private final double ARM_RELEASER_CLOSED = 0;
    private final double TOP_GRAB = 1;
    private final double TOP_UNGRAB = 0;
    private final double TOP_IDLE = .4;

    boolean shootMode = true;


    private final int SLEEP_CYCLE = 50;

    private boolean reversed;

    double powerL;
    double powerR;
    double shooterPower;
    double shooterIntegral;

    long currentTime = 0;
    protected double currentEncoder = 0;
    long lastTime = 0;
    double lastEncoder = 0;
    protected double velocity = 0.0;


    double[] velocityAvg = new double[90];
    int currentTick;
    double avg = 0;
    double lastAvg = 0;

    double voltage = 0.0;

    Runnable speedCounter = new Runnable() {
        @Override
        public void run() {
            currentTime = System.currentTimeMillis();
            currentEncoder = getShooterEncoderAvg();
            velocity = (currentEncoder - lastEncoder) / (currentTime - lastTime);
            telemetry.addData("time", currentTime * 1000);
            telemetry.addData("Encoder", currentEncoder + "--" + shooterL.getCurrentPosition() + "--" + shooterR.getCurrentPosition());
            if(velocity > 0) {
                velocityAvg[currentTick++] = velocity;
            }
            if(currentTick == 39) {
                lastAvg = avg;
                avg = 0;
                for(int i = 0; i < 40; i++) {
                    avg += velocityAvg[i];
                }
                avg /= 40;
                if(getShooterPower() > .04) {
//                    startPID(avg, lastTime);
                }
                currentTick = 0;
            }
            telemetry.addData("Velocity", avg);
            telemetry.update();
            lastTime = currentTime;
            lastEncoder = currentEncoder;
            try{
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    Thread speedThread;
    @Override
    public void init() {
        reversed = false;
        composeTelemetry();
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator = hardwareMap.dcMotor.get("mani");
        shooterR = hardwareMap.dcMotor.get("sR");
        shooterL = hardwareMap.dcMotor.get("sL");
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        activate = hardwareMap.servo.get("active");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        color = hardwareMap.colorSensor.get("colorL");
        color2 = hardwareMap.colorSensor.get("colorR");
        left = hardwareMap.opticalDistanceSensor.get("odsL");
        right = hardwareMap.opticalDistanceSensor.get("odsR");
        armLeft = hardwareMap.servo.get("aL");
        armRight = hardwareMap.servo.get("aR");
        liftRelease = hardwareMap.dcMotor.get("lrelease");
        armRelease = hardwareMap.servo.get("arelease");
        topGrabber = hardwareMap.servo.get("topGrabber");
        shooterPower = .3;
        shooterIntegral = 0;
        speedThread = new Thread(speedCounter);
        speedThread.start();
        frontIn();
        backIn();
        armsIn();
        activateShooter(false);
        unactivateLift();
        armBlocked();
        topUngrab();
        resetStartTime();
    }

    public void startMotors(double ri, double le) {
        if(reversed) {
            motorBL.setPower(-ri);
            motorFL.setPower(ri);
            motorBR.setPower(le);
            motorFR.setPower(-le);
        } else {
            motorBL.setPower(le);
            motorFL.setPower(-le);
            motorBR.setPower(-ri);
            motorFR.setPower(ri);
        }
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

//    public void pulseStop() throws InterruptedException {
//        while(powerL > .25 || powerR > .25) {
//            powerL /= 2;
//            powerR /= 2;
//            startMotors(powerR, powerL);
//            Thread.sleep(SLEEP_CYCLE);
//            stopMotors();
//            Thread.sleep(SLEEP_CYCLE * 2);
//        }
//    }

    public void frontOut() {
        front.setPosition(FRONT_OUT);
    }

    public void frontIn() {
        front.setPosition(FRONT_IN);
    }

    public void backOut() {
        back.setPosition(BACK_OUT);
    }

    public void backIn() {
        back.setPosition(BACK_IN);
    }

    public void reverse() {
        reversed = !reversed;
    }

    public void startMani() {
        manipulator.setPower(-1);
    }

    public void stopMani() {
        manipulator.setPower(0);
    }

    public void reverseMani() {
        manipulator.setPower(1);
    }

    public void startShooter() {
        openArms();
        topGrab();
        shooterL.setPower(shooterPower + shooterIntegral);
        shooterR.setPower(-shooterPower - shooterIntegral);
    }

    public void reverseShooter() {
        shooterL.setPower(-1);
        shooterR.setPower(1);
    }

    public void stopShooter() {
        shooterL.setPower(0);
        shooterR.setPower(0);
        shooterIntegral = 0;
        avg = 0;
        armsIn();
        topUngrab();
    }

    public double targetRPM(double pow) { //FILL THIS IN
        return 0.0;
    }

    void startPID(double avg, double lastTime) {
        double kP = .001;
        double kI = .015;
        double kD = .0011;

        double currentRPM = avg;
        double currentTime = System.currentTimeMillis();

        double error = targetRPM(getShooterPower()) - currentRPM;
        double proportional = error * kP;
        shooterIntegral += (currentTime * error * kI);
        double derivative = (currentRPM - lastAvg) / (currentTime - lastTime);
        shooterPower = proportional + shooterIntegral - derivative;

        startShooter();


    }

    public void activateShooter(boolean active) {
        if(active)
            activate.setPosition(1);
        else
            activate.setPosition(.2);
    }

    double getShooterPower() {
        return shooterPower;
    }

    public double getRightODS() {
        return right.getLightDetected();
    }

    public double getLeftODS() {
        return left.getLightDetected();
    }

    public double getRawRightODS() {
        return right.getRawLightDetected();
    }

    public double getRawLeftODS() {
        return left.getRawLightDetected();
    }

    public int getEncoderAvg() {
        return (Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorBL.getCurrentPosition()) +
                Math.abs(motorFR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition())) / 4;
    }

    public int getRightEncoderAvg() {
        return (Math.abs(motorFR.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition())) / 2;
    }

    public int getLeftEncoderAvg() {
        return (Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorBL.getCurrentPosition())) / 2;
    }

    public int getRed() {
        return color.red();
    }

    public int getBlue() {
        return color.blue();
    }

    public void armsIn() {
        armLeft.setPosition(1 - ARM_IN);
        armRight.setPosition(ARM_IN - .1);
    }

    public void grabArms() {
        armLeft.setPosition(1 - ARM_GRAB);
        armRight.setPosition(ARM_GRAB - .1);
    }

    public void openArms() {
        armLeft.setPosition(1- ARM_OPEN);
        armRight.setPosition(ARM_OPEN - .21);
    }

    public void dropArms() {
        armLeft.setPosition(ARM_DROP);
        armRight.setPosition(1 - ARM_DROP);
    }

    public void closeArms() {
        armLeft.setPosition(ARM_CLOSE);
        armRight.setPosition(1 - ARM_CLOSE);
    }

    public void armRelease() {
        armRelease.setPosition(ARM_RELEASER_RELEASED);
    }

    public void armBlocked() {
        armRelease.setPosition(ARM_RELEASER_CLOSED);
    }

    public void topGrab() {
        topGrabber.setPosition(TOP_GRAB);
    }

    public void topUngrab() {
        topGrabber.setPosition(TOP_UNGRAB);
    }

    public void topIdle() {
        topGrabber.setPosition(TOP_IDLE);
    }

    public void prepareLift() {
        dropArms();
        armRelease();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        openArms();
    }

    public int getShooterEncoderAvg() {
        return (Math.abs(shooterL.getCurrentPosition()) + Math.abs(shooterR.getCurrentPosition())) / 2;
    }

    public void activateLift() {
        liftRelease.setPower(.175);
    }

    public void unactivateLift() {
        liftRelease.setPower(0);
    }

    public double getVoltage() {
        return hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();
    }

    private void composeTelemetry() {
        telemetry.addLine()
                .addData("Shootermode", new Func<String>() {
                    @Override public String value() {
                        return "ShootMode: " + shootMode;
                    }
                });
        telemetry.addLine()
                .addData("ShooterPower", new Func<String>() {
                    @Override public String value() {
                        return "ShooterPower: " + getShooterPower();
                    }
                });
        telemetry.addLine()
                .addData("shooterRPow", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + shooterR.getPower();
                    }
                });
        telemetry.addLine()
                .addData("shooterLPow", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + shooterL.getPower();
                    }
                });
        telemetry.addLine()
                .addData("BL", new Func<String>() {
                    @Override public String value() {
                        return "BL: " + motorBL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("BR", new Func<String>() {
                    @Override public String value() {
                        return "BR: " + motorBR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FL", new Func<String>() {
                    @Override public String value() {
                        return "FL: " + motorFL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FR", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + motorFR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("shooterEncodersL", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + shooterL.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("shooterEncodersR", new Func<String>() {
                    @Override public String value() {
                        return "FR: " + shooterR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("releaseEncoder", new Func<String>() {
                    @Override public String value() {
                        return "release: " + liftRelease.getCurrentPosition();
                    }
                });

    }
}
