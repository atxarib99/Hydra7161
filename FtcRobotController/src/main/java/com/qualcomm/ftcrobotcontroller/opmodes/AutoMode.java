//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public abstract class AutoMode extends MyOpMode {
    private AutoMode.a a = null;
    private Thread b = null;
    private ElapsedTime c = new ElapsedTime();
    private volatile boolean d = false;
    public AdafruitIMU gyro;
    public int currentEncoder;
    public int BLencoder;
    public int BRencoder;
    public int FRencoder;
    public int FLencoder;
    public int BRnullEncoder;
    public int BLnullEncoder;
    public int FRnullEncoder;
    public int FLnullEncoder;
    public double currentAngle;
    public TouchSensor rts;
    public TouchSensor lts;
    private boolean hit;

    public AutoMode() {

    }

    public abstract void runOpMode() throws InterruptedException;

    public synchronized void waitForStart() throws InterruptedException {
        while(!this.d) {
            synchronized(this) {
                this.wait();
            }
        }

    }

    public void waitOneFullHardwareCycle() throws InterruptedException {
        this.waitForNextHardwareCycle();
        Thread.sleep(1L);
        this.waitForNextHardwareCycle();
    }

    public void waitForNextHardwareCycle() throws InterruptedException {
        synchronized(this) {
            this.wait();
        }
    }

    public void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    public boolean opModeIsActive() {
        return this.d;
    }

    public final void init() {
        this.a = new AutoMode.a(this);
        this.b = new Thread(this.a);
        this.b.start();
    }

    public void moveForward(double pow) {
        while(!hit) {
            startMotors(pow, pow);
            if(rts.isPressed() || lts.isPressed()) {
                hit = true;
            }
        }
        stopMotors();
    }

    public void myWait(int time) {
        try {
            wait(time);
        } catch (InterruptedException e) {
            RobotLog.e(e.getMessage());
        }

    }
    public void raiseLifts(double pow, int time) {
        ElapsedTime thisTime = new ElapsedTime();
        thisTime.startTime();
        while (thisTime.time() < time) {
            liftL.setPower(pow);
            liftR.setPower(-pow);
        }
        liftL.setPower(0);
        liftR.setPower(0);
        thisTime = new ElapsedTime();
    }



    public final void first() {
        motorBL = hardwareMap.dcMotor.get("BL");
        manipulator = hardwareMap.dcMotor.get("mani");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        climberSwitch = hardwareMap.servo.get("switch");
//        rightRatchet = hardwareMap.servo.get("ratchetR");
//        leftRatchet = hardwareMap.servo.get("ratchetL");
        rightPaddle = hardwareMap.servo.get("rPad");
        leftPaddle = hardwareMap.servo.get("lPad");
        rightPaddle.setPosition(0);//TODO: UPDATE THESE VALUES LATER
        leftPaddle.setPosition(1); //TODO: UPDATE THESE VALUES LATER
//        leftRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
//        rightRatchet.setPosition(0); //TODO: UPDATE THESE VALUES LATER
        climberSwitch.setPosition(.55);
        BLnullEncoder = 0;
        BRnullEncoder = 0;
        FRencoder = 0;
        FLencoder = 0;
        BRencoder = 0;
        BLencoder = 0;
        FLnullEncoder = 0;
        FRnullEncoder = 0;
        currentEncoder = 0;
        currentAngle = 0;
        hit = false;
        rts = hardwareMap.touchSensor.get("rts");
        lts = hardwareMap.touchSensor.get("lts");
        try {
            gyro = new AdafruitIMU(hardwareMap, "hydro"
                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMUAccel.BNO055_ADDRESS_A * 2) //By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMUAccel.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {

        }
    }
    public final void init_loop() {
    }

    public final void start() {
        this.d = true;
        synchronized(this) {
            this.notifyAll();
        }
    }

    public final void loop() {
        if(this.a.a()) {
            throw this.a.b();
        } else {
            synchronized(this) {
                this.notifyAll();
            }
        }
    }

    public final void stop() {
        this.d = false;
        if(!this.a.c()) {
            this.b.interrupt();
        }

        this.c.reset();

        while(!this.a.c() && this.c.time() < 0.5D) {
            Thread.yield();
        }

        if(!this.a.c()) {
            RobotLog.e("*****************************************************************");
            RobotLog.e("User Linear Op Mode took too long to exit; emergency killing app.");
            RobotLog.e("Possible infinite loop in user code?");
            RobotLog.e("*****************************************************************");
            System.exit(-1);
        }
        stopMotors();
        stopLifts();
        stopManipulator();

    }

    private static class a implements Runnable {
        private RuntimeException a = null;
        private boolean b = false;
        private final AutoMode c;

        public a(AutoMode var1) {
            this.c = var1;
        }

        public void run() {
            this.a = null;
            this.b = false;

            try {
                this.c.runOpMode();
            } catch (InterruptedException var6) {
                RobotLog.d("LinearOpMode received an Interrupted Exception; shutting down this linear op mode");
            } catch (RuntimeException var7) {
                this.a = var7;
            } finally {
                this.b = true;
            }

        }

        public boolean a() {
            return this.a != null;
        }

        public RuntimeException b() {
            return this.a;
        }

        public boolean c() {
            return this.b;
        }
    }
}
