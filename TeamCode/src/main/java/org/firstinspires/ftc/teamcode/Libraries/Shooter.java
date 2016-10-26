package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Lernaean.LernaeanAutoMode;
import org.firstinspires.ftc.teamcode.Lernaean.LernaeanOpMode;

/**
 * Created by Arib on 10/6/2016.
 */
public class Shooter {
    DcMotor shooterR;
    DcMotor shooterL;
    LinearOpMode opMode;

    private final String LOG_TAG = "Shooter";
    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        shooterL = this.opMode.hardwareMap.dcMotor.get("sR");
        shooterR = this.opMode.hardwareMap.dcMotor.get("sL");
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }
    public void startShooter(double sp){
        shooterL.setPower(sp);
        shooterR.setPower(-sp);
    }
    public void stopShooter(){
        shooterL.setPower(0);
        shooterR.setPower(-0);
    }
}
