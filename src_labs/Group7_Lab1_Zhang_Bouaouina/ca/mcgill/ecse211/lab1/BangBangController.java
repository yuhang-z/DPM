package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	/**
	 *Method that implements the bang bang controller
	 *Takes in info from the US sensor and uses it to control the robot 
	 *
	 *
	 *@param distance : distance from the wall detected by the US sensor
	 */
	public void processUSData(int distance) {
		
		this.distance = distance;
		
		int distError = this.bandCenter - this.distance;
		
		//if error is below threshold, keep the same speeds for the wheels
		if(Math.abs(distError) <= this.bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		/*if VERY close from the wall, decrease the speed of the outer wheel and increase greatly the speed of the outer wheel
		 *this condition was specifically added for the concave corner because the robot wouldn't have enough space to turn
		 *when it realizes that it is too close from the wall in front of it
		 */ 
		else if (Math.abs(distError) < 10) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh+60);
			WallFollowingLab.rightMotor.setSpeed(-30);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//too close to the wall 
		//higher speed for inner wheel
		else if(distError > 0){
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//too far from the wall 
		//higher speed for outer wheel
		else if(distError < 0) {
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} 
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
