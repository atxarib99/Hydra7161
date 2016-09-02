package org.firstinspires.ftc.teamcode;

/**
 * Created by Arib on 10/24/2015.
 */
public interface hydraDriveBase {
    void resetEncoders();
    void startMotors(double power1, double power2, double power3, double power4); //Assumes drivetrain has 4 motor drive
    void stopMotors();
    int getEncoderAvg();

}
