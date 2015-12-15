//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public abstract class AutoMode extends MyOpMode {
    private AutoMode.a a = null;
    private Thread b = null;
    private ElapsedTime c = new ElapsedTime();
    private volatile boolean d = false;

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

    public final void first() {
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        climberBar = hardwareMap.servo.get("climberBar");
        climberBar.setPosition(.55);
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
