package org.firstinspires.ftc.teamcode.Libraries;

/**
 * This class is designed to fix the deficiencies between the Modern Robotics platform and
 * the AndyMark 40 Motor in the RUN_WITHOUT_ENCODERS mode. Normally, the relationship between the value sent to the motor 
 * and the power the motor is run at is logarithmic. So if you send it a value of 0.25 for example,
 * the motor will run at about 80% power. When calling scaleSimple or scaleExact like this:
 * motorL.setPower(MotorScaler.scaleExact(gamepad1.right_stick_y));
 * at a right stick y value of 0.2, the motor would be run at 20% power.
 * The scaleControlled methods are intended to increase motor accuracy at lower powers.
 * This is most useful when driving a robot during the Driver Controlled period where accurate
 * controls are needed with a joystick, e.x. an arm that picks up things is being used.
 *
 * @author SpencerSharp
 * @version 1.0, Spring 2016
*/
 
public class MotorScaler {

	private MotorScaler()
	{
	
	}
	
	/**
 	* Returns a double that represents the "properly scaled" value from controller input to AndyMark 40 Motor.
 	* Essentially, makes the % of total mA for the motor sent the same as the % amount the
 	* joystick is pushed. For example, 0.25 sent would return a value that, when sent to the motor,
 	* runs the motor at 25% of its mA capacity. <Update> While this class may be too complex to analyze,
     * it should work as intended - Arib</Update>
 	* <p>
 	* This method is optimized to have minimal runtime, but is less accurate as a result.
 	* Use when lots of method runs per second (Setting motor power in teleop)
 	*
 	* @param  inpFromController  a double representing the value inputted from the controller
 	* @return a double representing the value sent to the motor in order to obtain the mA value
 	* @see    MotorScaler.scaleExact(double inpFromController)
 	*/
	public static double scaleSimple(double pwr)
	{
		if(pwr > 0.0)
        {
            if(pwr < 0.05) //0 PWR on chart
                return 0.0;
            else if(pwr >= 0.05 && pwr < 0.10) //0.05 on chart
                return 0.01;
            else if(pwr >= 0.10 && pwr < 0.15) //0.10 on chart
                return 0.02;
            else if(pwr >= 0.15 && pwr < 0.20) //0.15 on chart
                return 0.03;
            else if(pwr >= 0.20 && pwr < 0.25) //0.20 on chart
                return 0.04;
            else if(pwr >= 0.25 && pwr < 0.30) //0.25 on chart
                return 0.05;
            else if(pwr >= 0.30 && pwr < 0.35) //0.30 on chart
                return 0.06;
            else if(pwr >= 0.35 && pwr < 0.40) //0.35 on chart
                return 0.07;
            else if(pwr >= 0.40 && pwr < 0.45) //0.40 on chart
                return 0.075;
            else if(pwr >= 0.45 && pwr < 0.50) //0.45 on chart
                return 0.08;
            else if(pwr >= 0.50 && pwr < 0.55) //0.50 on chart
                return 0.09;
            else if(pwr >= 0.55 && pwr < 0.60) //0.55 on chart
                return 0.10;
            else if(pwr >= 0.60 && pwr < 0.65) //0.60 on chart
                return 0.113;
            else if(pwr >= 0.65 && pwr < 0.70) //0.65 on chart
                return 0.126;
            else if(pwr >= 0.70 && pwr < 0.75) //0.70 on chart
                return 0.14;
            else if(pwr >= 0.75 && pwr < 0.80) //0.75 on chart
                return 0.15;
            else if(pwr >= 0.80 && pwr < 0.85) //0.80 on chart
                return 0.19;
            else if(pwr >= 0.85 && pwr < 0.90) //0.85 on chart
                return 0.225;
            else
                return 1.0;
        }
        else
        {
            if(pwr > -0.05) //0 PWR on chart
                return 0.0;
            else if(pwr <= -0.05 && pwr > -0.10) //0.05 on chart
                return -0.01;
            else if(pwr <= -0.10 && pwr > -0.15) //0.10 on chart
                return -0.02;
            else if(pwr <= -0.15 && pwr > -0.20) //0.15 on chart
                return -0.03;
            else if(pwr <= -0.20 && pwr > -0.25) //0.20 on chart
                return -0.04;
            else if(pwr <= -0.25 && pwr > -0.30) //0.25 on chart
                return -0.05;
            else if(pwr <= -0.30 && pwr > -0.35) //0.30 on chart
                return -0.06;
            else if(pwr <= -0.35 && pwr > -0.40) //0.35 on chart
                return -0.07;
            else if(pwr <= -0.40 && pwr > -0.45) //0.40 on chart
                return -0.075;
            else if(pwr <= -0.45 && pwr > -0.50) //0.45 on chart
                return -0.08;
            else if(pwr <= -0.50 && pwr > -0.55) //0.50 on chart
                return -0.09;
            else if(pwr <= -0.55 && pwr > -0.60) //0.55 on chart
                return -0.10;
            else if(pwr <= -0.60 && pwr > -0.65) //0.60 on chart
                return -0.113;
            else if(pwr <= -0.65 && pwr > -0.70) //0.65 on chart
                return -0.126;
            else if(pwr <= -0.70 && pwr > -0.75) //0.70 on chart
                return -0.14;
            else if(pwr <= -0.75 && pwr > -0.80) //0.75 on chart
                return -0.15;
            else if(pwr <= -0.80 && pwr > -0.85) //0.80 on chart
                return -0.19;
            else if(pwr <= -0.85 && pwr > -0.90) //0.85 on chart
                return -0.225;
            else
                return -1.0;
        }
	}
	
	/**
 	* Returns a double that represents the "properly scaled" value from controller input to AndyMark 40 Motor.
 	* Essentially, makes the % of total mA for the motor sent the same as the % amount the
 	* joystick is pushed. For example, 0.25 sent would return a value that, when sent to the motor,
 	* runs the motor at 25% of its mA capacity.
 	* <p>
 	* Use this method for standard movement, or autonomous powers. This method is slower 
 	* in runtime, but is more precise. Use if your input value needs to exactly match
 	* match the output. (Autonomous one time set)
 	*
 	* @param  inpFromController  a double representing the value inputted from the controller
 	* @return a double representing the value sent to the motor in order to obtain the mA value
 	* @see    MotorScaler.scaleSimple(double inpFromController)
 	*/
	public double scaleExact(double inpFromController)
	{
		double valToMotor = 0.0;
        return 0.0;
	}
	
	/**
 	* Returns a double that represents the "exponentially scaled" value from controller input to
 	* the AndyMark 40 Motor. Essentially, makes the relationship between value from the controller
 	* and power the motor is run at into an exponential one. This makes it much easier to control at low speeds.
 	* The relationship isn't exactly exponential, but intended to resemble one.
 	* <p>
 	* This method should be used to control the robot when precise movement is needed. Also use this vs. 
 	* scaleControlledExact when multiple method calls per second are being made. (Ex. TeleOp driving)
 	*
 	* @param  inpFromController  a double representing the value inputted from the controller
 	* @return a double representing the value sent to the motor in order to obtain the mA value
 	* @see    MotorScaler.scaleControlledExact(double inpFromController)
 	*/
	public double scaleControlledSimple(double inpFromController)
	{
	    return 0.0;
	}
	
	/**
 	* Returns a double that represents the "exponentially scaled" value from controller input to
 	* the AndyMark 40 Motor. Essentially, makes the relationship between value from the controller
 	* and power the motor is run at into an exponential one. This makes it much easier to control at low speeds.
 	* The relationship isn't exactly exponential, but intended to resemble one.
 	* <p>
 	* This method should be used to control the robot when precise movement is needed. Also use this vs. 
 	* scaleControlledSimple when one or less method calls per second are needed, or extreme precision.
 	*
 	* @param  inpFromController  a double representing the value inputted from the controller
 	* @return a double representing the value sent to the motor in order to obtain the mA value
 	* @see    MotorScaler.scaleControlledSimple(double inpFromController)
 	*/
	public double scaleControlledExact(double inpFromController)
	{
	    return 0.0;
	}
}