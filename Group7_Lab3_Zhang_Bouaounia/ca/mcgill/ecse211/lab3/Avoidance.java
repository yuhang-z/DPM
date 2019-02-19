package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class provides methods to handle obstacle avoidance
 */
public class Avoidance extends Thread{

  /* private fields */
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  /* private constants */ 
  private static final double WHEEL_RAD = 2.1; // the radius of the wheel
  private static final double TRACK = 11.7; // the distance between two wheel center
  private static final int ANGLE_ADJUST = 90;
  private static final int DISTANCE_ADJUST = 20;
  private static final int DISTANCE_LARGE_ADJUST = 45;
  private static final int ANGLE_SUB_ADJUST = 10;


  /* class constructor */
  public Avoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

  }

  /**
   * This method takes in the distance measured by the US sensor and uses it to dete
   * @param distance
   * @param x
   * @param y
   */
  public void detection(int distance) {

    /*
     * If an object is within 10 cm, 
     * this block is accessed to guide the robot around the object
     */
    
    if (distance < 10) {
      /*
       * Although here we implement the turn angle and as a fix angle the
       * flag_check status at the end of this method will make the robot
       * adjust its traveling direction and distance based on the position
       * it is after avoiding the obstacle. In addition, the robot can
       * actually adjust its position based on the distance between the robot
       * and the obstacle, and the target destination after it return to the travelTo method. 
       * (Therefore it is actually "half wallFollower and half hard turn."
       */

      // stop the the robot first
      rightMotor.stop();
      leftMotor.stop();

      // adjust the angle a little bit to compensate the mistake of the
      // two motor stop() delay
      rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_SUB_ADJUST), true);
      leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_SUB_ADJUST), false);

      // firstly turn right 90 degree
      leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), true);
      rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), false);

      // secondly travel forward 25 cm
      leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_ADJUST), true);
      rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_ADJUST), false);

      // thirdly turn left 90 degree
      leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), true);
      rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), false);

      // fourthly travel forward to pass the obstacle
      leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_LARGE_ADJUST), true);
      rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_LARGE_ADJUST), false);

      // fifthly turn left 90 degree
      leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), true);
      rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), false);

      // then go forward 25 units
      leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_ADJUST), true);
      rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, DISTANCE_ADJUST), false);

      // turn 90 degrees to right, back the previous traveling direction
      leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), true);
      rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, ANGLE_ADJUST), false);

      // set the Navigation check flag to true, and means that does not
      // arrive at the target point yet,
      // and the avoidance function had been called
      Navigation.AVOIDANCE_FLAG = true;
    }

  }


}
