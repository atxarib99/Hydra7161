package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 10/6/2016.
 */
public class Manipulator {
    DcMotor manip;
    DcMotor magazine;
    LinearOpMode opMode;
    public Manipulator (LinearOpMode opMode){
        this.opMode = opMode;
        manip = this.opMode.hardwareMap.dcMotor.get("manip");
        magazine = this.opMode.hardwareMap.dcMotor.get("magazine");
    }

    public void runCollector(double pow){
        manip.setPower(pow);
    }

    public void runMagazine(double pow){
        magazine.setPower(pow);
    }
}
