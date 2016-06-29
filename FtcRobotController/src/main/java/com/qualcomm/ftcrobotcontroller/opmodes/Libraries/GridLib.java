package com.qualcomm.ftcrobotcontroller.opmodes.Libraries;

/**
 * Created by Arib on 5/9/2016.
 */


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

    public static void moveToCoordinatePos() {

    }

}
