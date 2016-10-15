package org.firstinspires.ftc.teamcode.Libraries;

/**
 * Created by Arib on 5/9/2016.
 */


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OldFiles.AdafruitIMU;

/**
 * This library is designed to allow any four consistently wheeled robot to do simple autonomous tasks
 * such as move across the field. Its required assets require 4 base wheel AndyMark Motors (or any
 * motor is coded the same way and returns similar encoder values) and encoder wires attached to each
 * motor. It creates encoder values based on which tile you want to move. Currently the robot will be
 * allowed to from tile to tile. Currently in progress is the addition of decimal tile movement.
 */
public class GridLib {

    public GridLib() {

    }

    //This method returns an array doubles
    public static double[] getValues(double wheelDiameter, double xDiff, double yDiff) {
        double[] returning = new double[2];
        final double DISTANCE_PER_ROTATION = wheelDiameter * Math.PI;

        double absyDiff = Math.abs(yDiff);
        double absxDiff = Math.abs(xDiff);

        double angleToTurn = Math.sin(absxDiff / absyDiff);

        returning[0] = angleToTurn;

        double oneTileInInches = 24;                                                //encoder value calculation for one tile
        Double xdistToMoveInches = oneTileInInches * xDiff;                          //encoder value calculation for multiple tiles
        double xRotationsToMove = xdistToMoveInches / DISTANCE_PER_ROTATION;          //cast the encoder value as an integer
        double xencoderTicksToMoveSquared = Math.pow(Math.round(xRotationsToMove * 1120), 2);

        //calculate the amount of ticks for the other movement
        Double ydistToMoveInches = oneTileInInches * yDiff;                          //encoder value calculation for multiple tiles
        double yRotationsToMove = ydistToMoveInches / DISTANCE_PER_ROTATION;          //cast the encoder value as an integer
        double yencoderTicksToMoveSquared = Math.pow(Math.round(yRotationsToMove * 1120), 2);

        int hypotenuseTicks = (int) Math.sqrt(xencoderTicksToMoveSquared + yencoderTicksToMoveSquared); //use pythagorean theorem to calculate the

        returning[1] = hypotenuseTicks;

        return returning;
    }

    public static void moveToCoordinatePos(AdafruitIMU imu, double[] values, DcMotor... motors) {
        DcMotor motorBL = motors[0];
        DcMotor motorBR = motors[1];
        DcMotor motorFL = motors[2];
        DcMotor motorFR = motors[3];
        AdafruitIMU gyro = imu;
        double[] vals = values;
        double currentAngle = 0.0;
        while(currentAngle < vals[0]) {
            motorBL.setPower(.3);
            motorBR.setPower(-.3);
            motorFL.setPower(.3);
            motorFR.setPower(-.3);
            //Add gyro stuff

        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        int currentEncoder = 0;
        while(currentEncoder < vals[1]) {

        }
        //100k LINES OF CODE!!!


    }

}
