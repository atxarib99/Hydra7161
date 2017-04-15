package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 10/6/2016.
 */
public class Shooter {
    public DcMotor shooterR;
    DcMotor shooterL;
    LinearOpMode opMode;

    private final String LOG_TAG = "Shooter";
    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        shooterL = this.opMode.hardwareMap.dcMotor.get("sR");
        shooterR = this.opMode.hardwareMap.dcMotor.get("sL");
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }
    public void startShooter(double sp){
        shooterL.setPower(sp);
        shooterR.setPower(-sp);
    }

    public boolean startShooterRPM(int RPM) {
        double flyRPM;
        double oldFly = 0;
        double calculatedRPM = 0;
        double pastCalculatedRPM = 0;

        double[] flyArray = new double[100];
        int flyTicks = 0;
        while(calculatedRPM < 650 || calculatedRPM > 750) {
            flyRPM = (((shooterL.getCurrentPosition() + shooterR.getCurrentPosition()) / 2) - oldFly) / opMode.getRuntime();

            oldFly = ((shooterL.getCurrentPosition() + shooterR.getCurrentPosition()) / 2);

            if(flyTicks > 99) {
                double sum = 0;
                for(int i = 0; i < 100; i++) {
                    sum += flyArray[i];
                }
                calculatedRPM = (pastCalculatedRPM * .7) + ((sum / 100) * .3);
                pastCalculatedRPM = calculatedRPM;
                flyTicks /= 100;
            } else {
                flyArray[flyTicks] = flyRPM;
                flyTicks++;
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        return true;
    }

    public double getNeededPower(double voltage) {
        if(voltage > 14.1)
            return .285;
        if(voltage > 14 && voltage < 14.1)
            return .30;
        if(voltage < 14 && voltage > 13.75)
            return .325;
        if(voltage < 13.75 && voltage > 13.5)
            return .335;
        if(voltage < 13.5 && voltage > 13)
            return .350;
        if(voltage < 13 && voltage > 12)
            return .4;
        if(voltage < 12)
            return .43;

        return 0;
    }
    public void stopShooter(){
        shooterL.setPower(0);
        shooterR.setPower(0);
    }
}
