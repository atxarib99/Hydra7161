package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
    private Servo activate;
    Servo armRelease;
    Servo topGrabber;
    Servo topGrabberRunner;
    DeviceInterfaceModule cdim;
    private ColorSensor color;
    private ColorSensor color2;
    private OpticalDistanceSensor left;
    private OpticalDistanceSensor right;
    LernaeanOpMode opMode;


    private final double BACK_OUT = 0;
    private final double BACK_IN = 1;
    private final double FRONT_OUT = 0;
    private final double FRONT_IN = 1;
    private final double ARM_IN = .85;
    private final double ARM_GRAB = .68;
    private final double ARM_OPEN = .425;
    private final double ARM_DROP = .25;
    private final double ARM_CLOSE = 0;
    private final double ARM_RELEASER_RELEASED = 1;
    private final double ARM_RELEASER_CLOSED = 0;
    private final double TOP_GRAB = 1;
    private final double TOP_UNGRAB = 0;
    private final double TOP_IDLE = .4;

    boolean shootMode = true;

    private final int SLEEP_CYCLE = 50;

    private boolean reversed;

    Runnable speedCounter = new Runnable() {

        private double[] velocityAvg = new double[400];
        private int currentTick;
        private double avg = 0;
        private double lastAvg = 0;

        long lastTime = 0;
        int lastEncoder = 0;
        int currentEncoder;
        double velocity;
        @Override
        public void run() {
            while (Thread.currentThread().isAlive()) {
                long currentTime = System.nanoTime();
                currentEncoder = getShooterEncoderAvg();
                velocity = (currentEncoder - lastEncoder) / ((currentTime - lastTime) / 1000000);
                opMode.telemetry.addData("velocity", velocity);
                opMode.telemetry.addData("time", currentTime);
                opMode.telemetry.addData("Encoder", currentEncoder + "--" + shooterL.getCurrentPosition() + "--" + shooterR.getCurrentPosition());
                if (velocity > 0) {
                    velocityAvg[currentTick++] = velocity;
                }
                opMode.telemetry.addData("currentTick", currentTick);
                if (currentTick == 399) {
                    lastAvg = avg;
                    avg = 0;
                    for (int i = 0; i < 400; i++) {
                        avg += velocityAvg[i];
                    }
                    avg /= 40;
                    if (getShooterPower() > .04) {
//                    startPID(avg, lastTime);
                    }
                    currentTick = 0;
                }
                opMode.telemetry.addData("avgVelocity", avg);
                opMode.telemetry.update();
                lastTime = currentTime;
                lastEncoder = currentEncoder;
                try {
                    Thread.sleep(0, 10000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    double powerL;
    double powerR;
    double shooterPower;
    double shooterIntegral;

    double voltage = 0.0;

    private Runnable ABSright = new Runnable() {

        double kP = .1;
        double kI = .1;
        double kD = .2;
        double lastPower = 0;
        private long lastTime = 0;
        double ABSRightStartTime = System.currentTimeMillis();

        @Override
        public void run() {
            double target = gamepad1.right_stick_y;
            long currentTime = System.currentTimeMillis();

            double error = target - motorBR.getPower();
            double proportional = error * kP;
            double integral = ((currentTime - ABSRightStartTime) * error * kI);
            double derivative = ((motorBR.getPower() - lastPower) / (currentTime - lastTime)) * kD;

            double power = proportional + integral - derivative;
            lastPower = motorBR.getPower();
            lastTime = currentTime;

        }
    };

    private Runnable ABSleft = new Runnable() {

        double kP = .1;
        double kI = .01;
        double kD = .2;
        double lastPower = 0;
        double lastTime = 0;

        double ABSLeftStartTime = System.currentTimeMillis();

        @Override
        public void run() {
            double targetRight = gamepad1.left_stick_y;
            double errorRight = targetRight - motorBL.getPower();
            double currentTime = System.currentTimeMillis();

            while(Math.abs(errorRight) > .02) {
                double proportional = errorRight * kP;
                double integral = ((currentTime - ABSLeftStartTime) * errorRight * kI);
                double derivative = ((motorBL.getPower() - lastPower) / (currentTime - lastTime)) * kD;

                double power = proportional + integral - derivative;
                startMotors(power, power);
                lastPower = motorBL.getPower();
                lastTime = currentTime;
            }

        }
    };

    private Thread ABSRightThread;
    private Thread ABSLeftThread;
    private Thread speedThread;
    @Override
    public void init() {
        opMode = this;
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
        liftRelease = hardwareMap.dcMotor.get("lrelease");
        armRelease = hardwareMap.servo.get("arelease");
        topGrabber = hardwareMap.servo.get("topGrabber");
        topGrabberRunner = hardwareMap.servo.get("topGrabRun");
        shooterPower = .3;
        shooterIntegral = 0;
        speedThread = new Thread(speedCounter);
        ABSLeftThread = new Thread(ABSleft);
        ABSRightThread = new Thread(ABSright);
        frontIn();
        backIn();
        topIdle();
        topGrabIdle();
        activateShooter(false);
        unactivateLift();
        armBlocked();
        topIdle();
        resetStartTime();
    }

    @Override
    public void stop() {
//        speedThread.interrupt();
    }

    @Override
    public void start() {
//        speedThread.start();
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

    @Deprecated
    public void pulseStop() throws InterruptedException {
        while(powerL > .25 || powerR > .25) {
            powerL /= 2;
            powerR /= 2;
            startMotors(powerR, powerL);
            Thread.sleep(SLEEP_CYCLE);
            stopMotors();
            Thread.sleep(SLEEP_CYCLE * 2);
        }
    }

    void frontOut() {
        front.setPosition(FRONT_OUT);
    }

    void frontIn() {
        front.setPosition(FRONT_IN);
    }

    void backOut() {
        back.setPosition(BACK_OUT);
    }

    void backIn() {
        back.setPosition(BACK_IN);
    }

    public void reverse() {
        reversed = !reversed;
    }

    void startMani() {
        manipulator.setPower(-1);
    }

    void stopMani() {
        manipulator.setPower(0);
    }

    void reverseMani() {
        manipulator.setPower(1);
    }

    public void startShooter() {
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
        topUngrab();
    }

    @Deprecated
    private double targetRPM(double pow) {
        return 0.0;
    }

    public void activateShooter(boolean active) {
        if(active)
            activate.setPosition(1);
        else
            activate.setPosition(.2);
    }

    private double getShooterPower() {
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

    void armRelease() {
        armRelease.setPosition(ARM_RELEASER_RELEASED);
    }

    void armBlocked() {
        armRelease.setPosition(ARM_RELEASER_CLOSED);
    }

    void topGrab() {
        topGrabber.setPosition(1);
    }

    void topUngrab() {
        topGrabber.setPosition(-1);
    }

    void topIdle() {
        topGrabber.setPosition(.5);
    }

    void topGrabRun() {
        topGrabberRunner.setPosition(-1);
    }

    void topGrabReverse() {
        topGrabberRunner.setPosition(1);
    }

    void topGrabIdle() {
        topGrabberRunner.setPosition(.5);
    }

    void prepareLift() {
        armRelease();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private int getShooterEncoderAvg() {
        return (Math.abs(shooterL.getCurrentPosition()) + Math.abs(shooterR.getCurrentPosition())) / 2;
    }

    void activateLift() {
        liftRelease.setPower(1);
    }

    void unactivateLift() {
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
                .addData("voltage", new Func<String>() {
                    @Override public String value() {
                        return "voltage: " + voltage;
                    }
                });
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
