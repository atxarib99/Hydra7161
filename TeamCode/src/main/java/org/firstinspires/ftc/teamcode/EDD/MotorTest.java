package org.firstinspires.ftc.teamcode.EDD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Arib on 1/10/2017.
 */

@Autonomous(name = "EDDMOTORTEST", group = "EDD")
public class MotorTest extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("R");
        motor2 = hardwareMap.dcMotor.get("L");

        waitForStart();

        while(opModeIsActive()) {
            motor1.setPower(1);
            motor2.setPower(1);
        }

        motor1.setPower(0);
        motor2.setPower(0);
    }
}
