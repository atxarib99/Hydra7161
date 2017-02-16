package org.firstinspires.ftc.teamcode.EDD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 2/15/2017.
 */

@TeleOp(name = "LIGHTS", group = "DEVAN")
public class LightTest extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("lights1");
        motor2 = hardwareMap.dcMotor.get("lights2");
    }

    @Override
    public void loop() {
        motor1.setPower(1);
        motor2.setPower(1);
    }
}
