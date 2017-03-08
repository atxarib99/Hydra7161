package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    Servo armLeft;
    Servo armRight;
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
    private final double ARM_RELEASER_RELEASED = 0;
    private final double ARM_RELEASER_CLOSED = 1;
    private final double TOP_GRAB = 1;
    private final double TOP_UNGRAB = 0;
    private final double TOP_IDLE = .4;

    boolean shootMode = true;

    private final int SLEEP_CYCLE = 50;

    private boolean reversed;
    boolean stopCommandGiven;

    Runnable speedCounter = new Runnable() {

        private double[] velocityAvg = new double[20];
        private int currentTick;
        private double avg = 0;
        private double lastAvg = 0;
        private int numZero = 0;
        private double power;

        long lastTime = 0;
        int lastEncoder = 0;
        int currentEncoder;
        double velocity;
        @Override
        public void run() {
            while (Thread.currentThread().isAlive()) {
                long currentTime = System.nanoTime();
                currentEncoder = getShooterEncoderAvg();
                try {
                    velocity = (currentEncoder - lastEncoder) / ((currentTime - lastTime) / 1000000);
                } catch (ArithmeticException e) {
                    velocity = 0;
                }
                opMode.telemetry.addData("velocity", velocity);
                opMode.telemetry.addData("time", currentTime);
                opMode.telemetry.addData("Encoder", currentEncoder + "--" + shooterL.getCurrentPosition() + "--" + shooterR.getCurrentPosition());
                if (velocity > 0) {
                    velocityAvg[currentTick++] = velocity;
                } else {
                    if(shooterL.getPower() > 0)
                        numZero++;
                }
                opMode.telemetry.addData("currentTick", currentTick);
                if (currentTick == 19) {
                    lastAvg = avg;
                    avg = 0;
                    for (int i = 0; i < 20; i++) {
                        avg += velocityAvg[i];
                    }
                    avg /= 20;
                    double error = 1.8 - avg;
                    if (getShooterPower() > .04 && Math.abs(error) > .2 && !stopCommandGiven) {
                        double kP = 1.2; //12.5
                        power = kP * error;
                        power = Range.clip(power, 0, .6); //0, .65
                        if(power == 0) {
                            power = getShooterPower();
                        }
                        shooterL.setPower(power);
                        shooterR.setPower(-power);
                    }
                    currentTick = 0;
                }
                opMode.telemetry.addData("Number Of Zeros", numZero);
                opMode.telemetry.addData("avgVelocity", avg);
                opMode.telemetry.update();
                lastTime = currentTime;
                lastEncoder = currentEncoder;
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                opMode.telemetry.addData("power", power);
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
        stopCommandGiven = true;
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
        armLeft = hardwareMap.servo.get("aL");
        armRight = hardwareMap.servo.get("aR");
        shooterPower = .3;
        shooterIntegral = 0;
        speedThread = new Thread(speedCounter);
        ABSLeftThread = new Thread(ABSleft);
        ABSRightThread = new Thread(ABSright);
        frontIn();
        backIn();
        armsIn();
        activateShooter(false);
        unactivateLift();
        armBlocked();
        resetStartTime();
    }

    @Override
    public void stop() {
        speedThread.interrupt();
    }

    @Override
    public void start() {
        speedThread.start();
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

    public void startMotorsSlowed(double ri, double le) {
        if((ri < -.05 && le < -.05) || (ri > .05 && le > .05)) {
            ri /= .375;
            le /= .375;
        }
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

    void armsIn() {
        armLeft.setPosition(.5);
        armRight.setPosition(.5);
    }

    void armsOut() {
        armLeft.setPosition(.37);
        armRight.setPosition(.63);
    }

    void armsDrop() {
        armLeft.setPosition(0);
        armRight.setPosition(1);
    }

    void armsGrab() {
        armLeft.setPosition(.53);
        armRight.setPosition(.47);
    }

    void prepareLift() {
        armsDrop();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        armsOut();
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
