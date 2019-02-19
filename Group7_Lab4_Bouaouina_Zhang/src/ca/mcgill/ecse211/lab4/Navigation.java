package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class provides methods to drive the robot to a certain.
 * 
 * @author Mohamed Youssef Bouaouina
 * @author YuHang Zhang
 */
public class Navigation {


  /* private constants */
  private static final double TILE_SIZE = 30.48;
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATION_SPEED = 75;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  /* private variables */
  private double d_x;
  private double d_y;

  private double angle = 0; // the heading angle

  // variables represent current x, y, theta
  private double currentX;
  private double currentY;
  private double currentT;

  // status_sensor of the turning and moving
  private boolean isTurning;
  private boolean isMoving;

  // get the sampling from the odometer;
  private Odometer odometer;

  /*
   * Navigation constructor
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
   }

  /**
   * TravelTo function takes as arguments the coordinates of the desired destination. 
   *  
   * @param x : double representing the x coordinate of the destination point
   * @param y : double representing the y coordinate of the destination point
   */
  public void travelTo(double x, double y) {

    // set the status of robot to moving
    isMoving = true;

    // get current position of the x, y and traveling direction
    currentX = odometer.getXYT()[0];
    currentY = odometer.getXYT()[1];
    currentT = odometer.getXYT()[2];

    // get the distance need to travel to set coordinate x, y
    d_x = (x * TILE_SIZE)- currentX;
    d_y = (y * TILE_SIZE) - currentY;

    // check the direction need to travel to (represented by the angle)
    // need to travel forward
    if (d_y >= 0) {
      angle = Math.atan(d_x / d_y);
    }
    // need to travel backward right direction
    else if (d_y < 0 && d_x >= 0) {
      angle = Math.PI + Math.atan(d_x / d_y);
    }
    // need to travel to backward left direction
    else {
      angle = Math.atan(d_x / d_y) - Math.PI;
    }
    // transfer the angle from radian to the degree
    double turnangle = radianToDegree(angle) - currentT;

    // turn the direction of traveling to the target direction
    turnTo(turnangle);

    // travel in the direction for a target distance and forward speed
    double distance = Math.sqrt(d_x * d_x + d_y * d_y);

    // set the speed of the motor
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // set the distance of traveling as the distance from current position
    // to target position
    leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), true);
    rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), true);

    isMoving = false;
  }

  /**
   * This method turns the robot to the inputed angle.
   * Used to place the robot in the right destination heading
   * 
   * @param angle : heading angle to turn to
   */
  public void turnTo(double theta) {
    // set the status_sensor of the turning to true
    isTurning = true;
    // check and adjus_sensort the turning angle, and choose the correct
    // minimal angle to turn: always turn an angle less than 180, 
    //and greater than zero
    if (theta > 180) {
      theta = 360 - theta;
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);
      leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
      rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
    } else if (theta < -180) {
      theta = theta + 360;
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);
      leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
      rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
    } else {
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);
      leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
      rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
    }
    // reset the status_sensor of turning
    isTurning = false;

  }

  /**
   * Method that determines if the robot is navigating
   * 
   * @return isTurning||isMoving
   */
  public boolean isNavigating() {
    if (isTurning == true || isMoving == true) {
      return true;
    }
    return false;
  }

  /**
   * Helper method that converts the passed angle from radian to degrees
   * 
   * @param theta : double representing the angle in radian to be converted
   * @return double representing the converted angle 
   */
  public double radianToDegree(double theta) {
    return theta * 180 / Math.PI;
  }

  /**
   * Helper method that converts the traveling distance to the number of degrees the wheel of the 
   * robot has to rotate
   *  
   * @param radius of the wheel 
   * @param distance to travel
   * 
   * @return number of degrees
   */
  public static int convertDistance(double wheelRadius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * wheelRadius));
  }

  /**
   * Helper method that converts the traveling angle to the number of degrees the wheel of the 
   * robot has to rotate
   * 
   * @param wheelRadius: radius of the wheel
   * @param width : the distance between two wheels
   * @param angle to rotate to 
   * 
   * @return numbers of degrees
   */

  public static int convertAngle(double wheelRadius, double width, double angle) {
    return convertDistance(wheelRadius, Math.PI * width * angle / 360.0);
  }

}
