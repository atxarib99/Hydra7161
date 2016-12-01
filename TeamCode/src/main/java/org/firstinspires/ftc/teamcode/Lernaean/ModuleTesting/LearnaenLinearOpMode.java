package org.firstinspires.ftc.teamcode.Lernaean.ModuleTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Libraries.BeaconPushers;
import org.firstinspires.ftc.teamcode.Libraries.Drivetrain;
import org.firstinspires.ftc.teamcode.Libraries.Manipulator;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

/**
 * Created by Arib on 11/23/2016.
 */

public abstract class LearnaenLinearOpMode extends LinearOpMode {

    Drivetrain drivetrain;
    Manipulator manipulator;
    Shooter shooter;
    BeaconPushers beaconPushers;

    public double shooterPower = .3;
    public boolean reversed = false;

    public double getShooterPower() {
        return shooterPower;
    }

    public void composeTelemetry() {
        telemetry.addLine()
                .addData("ShooterPower", new Func<String>() {
                    @Override public String value() {
                        return "ShooterPower: " + getShooterPower();
                    }
                });
    }

}
