/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;
/**
 * Group_7 YuhangZhang YoussefMohamed
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;


	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/*private constants*/
	private double dx;   //displacement at x direction 
	private double dy;   //displacement at x direction 
	private double dLeft;
	private double dRight;
	private int preleft;
	private int preright;
	private double dDistance;
	private double dT;
	private double theta;
	private double X = 0;
	private double Y = 0;

	//------------------


	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
		// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
					throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be used only if an
	 * odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();
			//use the formula degree/180*number_wheel_rotate*PI to count the distance
			//traveling of the left and right wheels;
			dLeft = Math.PI * WHEEL_RAD * (leftMotorTachoCount -preleft) / 180;    //distance passed of the left motor  
			dRight = Math.PI * WHEEL_RAD * (rightMotorTachoCount -preright) / 180;   //right 
			preleft = leftMotorTachoCount;   //the left distance in the last run on the left motor 
			preright = rightMotorTachoCount;  // same thing at the right motor 
			dDistance = (dLeft+dRight) / 2;   //actual distance of the displacement of the robot
			dT = (dLeft-dRight) / TRACK;     //the angle of turning 
			theta += dT;                     //accumulated angle 
			dx = Math.sin(theta) * dDistance;   //displacement in x direction 
			dy = Math.cos(theta) * dDistance;	  //displacement in y direction  
			X = X + dx; Y = Y + dy;     //the actual plan
			odo.update(dx, dy, dT*180/Math.PI);

			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}
