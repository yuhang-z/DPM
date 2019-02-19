package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 40;
	private static final int PROP_GAIN = 6; //p-controller constant
	
	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;
	

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	/**
	 *Method that implements the p-controller
	 *Takes in info from the US sensor and uses it to control the robot 
	 *
	 *
	 *@param distance : distance from the wall detected by the US sensor
	 */
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		int wallError = 26 - this.distance;

		//error is within threshold, keep going straight
		if(Math.abs(wallError) <= this.bandWidth) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();    	
		}
		//too close to the wall
		else if(wallError > 0) {
			/*
			 * if error is too big, set it to a specific value
			 * this if statement was added to prevent the robot from spiraling out of control
			 * when the wallError is very large 
			 */
			if(wallError > 25) {
				wallError = 10;
			}
			int deltaSpeed = PROP_GAIN * Math.abs(wallError); //compute the adjustment speed based on the error
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + deltaSpeed); 
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		else if(wallError < 0){
			/*
			 * if error is too big, set it to a specific value
			 * this if statement was added to prevent the robot from spiraling out of control
			 * when the wallError is very large 
			 */
			if(wallError < -25) {
				wallError = -10;
			}
			int deltaSpeed = PROP_GAIN * Math.abs(wallError); //compute the adjustment speed based on the error
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - deltaSpeed); 
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}


	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
