package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 4/1/2017.
 */

@Autonomous(name = "GetRunTimeTest", group = "LinearOpMode")
public class GetRunTimeTest extends LinearOpMode {
    ElapsedTime time;
    long startTime;
    @Override
    public void runOpMode() throws InterruptedException {
        time = new ElapsedTime();
        waitForStart();
        resetStartTime();
        startTime = System.nanoTime();
        time.startTime();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                Thread.sleep(1000);
            }
            if(gamepad1.b) {
                Thread.sleep(500);
            }
            doTelemetry();
        }
    }

    private void doTelemetry() {
        telemetry.addData("ElapsedTime", time.seconds());
        telemetry.addData("GetRunTime", getRuntime());
        telemetry.addData("CalculatedTime", ((System.nanoTime() - startTime) * 1000000000L));
        telemetry.update();
    }
}
