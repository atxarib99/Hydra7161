package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import android.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Lift;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 2/21/2017.
 */

@Autonomous(name = "Judging", group = "LinearOpMode")
public class Judging extends LinearOpMode {
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Shooter shooter;
    private BeaconPushers beaconPushers;
    private Lift lift;
    private double voltage;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(this);
        lift.prepareLift();

        waitForStart();

        lift.grabArms();

        Thread.sleep(2000);

        lift.activateLift();

        Thread.sleep(3000);

        lift.unactivateLift();

        while(opModeIsActive());

        lift.openArms();

        Thread.sleep(1500);
    }
}