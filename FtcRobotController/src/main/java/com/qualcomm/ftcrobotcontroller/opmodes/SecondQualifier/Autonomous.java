package com.qualcomm.ftcrobotcontroller.opmodes.SecondQualifier;

import com.qualcomm.ftcrobotcontroller.opmodes.AutoMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Arib on 1/4/2016.
 */
public class Autonomous extends AutoMode {
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() {
        first();
        time.startTime();
        while(time.time() < 1) {
            startMotors(1, -1);
        }
        while(opModeIsActive()) {
            startMotors(1, 1);
        }
    }
}
