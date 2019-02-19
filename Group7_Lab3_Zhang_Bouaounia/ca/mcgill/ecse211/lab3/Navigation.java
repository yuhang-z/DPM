package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * Navigation Class used to drive the robot some target points following a required path
 * Navigation Class includes methods: travelTo, causes the robot to travel to the absolute field location 
 * turnTo, a method decide which direction the robot should go 
 * isNavigating, a method set the status of the robot activities
 * 
 */
/**
 * This class provides methods to drive the robot.
 * The methods ensure that the robot passes through every waypoint.
 * 
 * @author Mohamed Youssef Bouaouina
 * @author YuHang Zhang
 */
public class Navigation extends Thread {


  /* private constants */
  private static final double TILE_SIZE = 30.48;
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATION_SPEED = 100;
  private static final double TRACK = 11.7;
  private static final double WHEEL_RAD = 2.1;
  public static boolean AVOIDANCE_FLAG = false;
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
  private SampleProvider us_sensor;
  private Avoidance avoidance;
  private float[] us_sensorData;
  private int[][] waypoints;

  /*
   * Navigation constructor
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int[][] waypoints, 
      Odometer odometer, SampleProvider us_sensor, float[] us_sensorData, Avoidance avoidance) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.us_sensor = us_sensor;
    this.us_sensorData = us_sensorData;
    this.avoidance = avoidance;
    this.waypoints = waypoints;
  }

  public void run() {
    // for the all five points in the lists
    for (int[] point : this.waypoints) {
      try {
        // thread sleep and then run to the next position
        Thread.sleep(1200);
      } catch (InterruptedException e) {
      }
      /*
       * call function travel to, travel to next point coordinate point is
       * separated by a square_size (30.48cm)
       */
      travelTo(point[0] * TILE_SIZE, point[1] * TILE_SIZE);
    }
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
    d_x = x - currentX;
    d_y = y - currentY;

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
    leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, distance), true);

    // run the avoidance obstacle when the robot is moving
    int obstacle_check;
    //the obstacle check is running all the time to check the obstacle in the traveling path
    while (leftMotor.isMoving() == true || rightMotor.isMoving() == true) {

      // get the data from the ultrasonic sensor
      us_sensor.fetchSample(us_sensorData, 0);
      // get the latest detection data
      obstacle_check = (int) (us_sensorData[0] * 100.0);

      // call the detection to decide whether the path need to be changed
      avoidance.detection(obstacle_check);

      try {
        Thread.sleep(30);
      } catch (Exception e) {
      }

    }
    // if the avoidance.detection function is used
    // and the robot does not arrive at the target point; after avoiding the
    // obstacle go the target point
    if (AVOIDANCE_FLAG == true) {
      AVOIDANCE_FLAG = false;
      travelTo(x, y);
    }
    // change the status of the robot
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
      leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
    } else if (theta < -180) {
      theta = theta + 360;
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
    } else {
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
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
