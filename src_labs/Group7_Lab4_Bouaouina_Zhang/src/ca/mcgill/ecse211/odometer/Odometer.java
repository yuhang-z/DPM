package ca.mcgill.ecse211.odometer;

/**
 * This class is meant as a skeleton for the odometer class to be used. * 
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
  private double dx;
  private double dy;
  private double dLeft;
  private double dRight;
  private int previous_leftTacho;
  private int previous_rightTacho;
  private double dDistance;
  private double dT;
  private double angle_theta;
  private double X=0;
  private double Y=0;
  
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
      dLeft=Math.PI*WHEEL_RAD*(leftMotorTachoCount-previous_leftTacho)/180;
      dRight=Math.PI*WHEEL_RAD*(rightMotorTachoCount-previous_rightTacho)/180;
      
      //save the previous Tacho number 
      previous_leftTacho=leftMotorTachoCount;
      previous_rightTacho=rightMotorTachoCount;
      
      //calculate the mean of the left and right wheels traveling distance
      //to get the total distance traveling of the car
      dDistance=(dLeft+dRight)/2;
      
      //the angle of change during each adjustment (turn)
      dT=(dLeft-dRight)/TRACK;
      //the total angle change of the robot
      angle_theta+=dT;
      
      //when the robot change direction x, y coordinates 
      //change with the angle of theta and the distance it travel;
      dx=Math.sin(angle_theta)*dDistance;
      dy=Math.cos(angle_theta)*dDistance;
      //get the new x and y coordinates based on the change 
      X=X+dx;
      Y=Y+dy;
      
      // update the x y and the angle (units: cm cm degree)
      odo.update(dx, dy, dT*180/Math.PI);

      // this ensures that the odometer only runs once every period
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
