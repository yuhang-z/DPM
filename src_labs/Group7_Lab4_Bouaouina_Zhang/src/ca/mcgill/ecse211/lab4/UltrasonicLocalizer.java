package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class provides methods to perform ultrasonic localization. 
 * 2 versions of the localization are available: Rising and Falling Edge
 * 
 * @author Mohamed Youssef Bouaouina
 * @author Yuhang Zhang
 */
public class UltrasonicLocalizer {

  //private constants
  public enum LocalizationVersion { FALLING_EDGE, RISING_EDGE };
  public static final int ROTATION_SPEED = 50;
  private Odometer odo;
  private SampleProvider usSensor;
  private Navigation navigation;
  private float[] usData;
  private LocalizationVersion locVersion;
  private static final double DISTANCE_THRESHOLD = 25;

  /**
   * Class constructor that creates an UltrasonicLocalizer object
   * 
   * @param odo : Odometer object used for determining coordinates of the robot
   * @param usSensor : SampleProvider of the ultrasonic sensor
   * @param usData : float array to hold US sensor data
   * @param locVersion : LocalizationVersion chosen by user
   * @param navigation : Navigation object that provides navigation methods
   */
  public UltrasonicLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationVersion locVersion, Navigation navigation) {
    this.odo = odo;
    this.usSensor = usSensor;
    this.usData = usData;
    this.locVersion = locVersion;
    this.navigation = navigation;
  }
  /**
   * Method that performs the ultrasonic localization. 
   * The routine to be performed (Falling or Rising) is determined by the locVersion instance variable
   * of the UltrasonicLocalization object this method is called on 
   */
  public void doLocalization() {
    double angleA, angleB, angleAvg;

    Lab4.leftMotor.setSpeed(ROTATION_SPEED);
    Lab4.rightMotor.setSpeed(ROTATION_SPEED);

    //for falling edge localization
    if (locVersion == LocalizationVersion.FALLING_EDGE) {

      /* if the robot starts facing a wall, turn it away from the wall
       * then the falling edge procedure starts properly
       */
      if (getFilteredData() <= DISTANCE_THRESHOLD) {
        Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 180), true);
        Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 180), false);

      }

      //start falling edge procedure by turning robot
      Lab4.leftMotor.forward();
      Lab4.rightMotor.backward();

      /* continuously check for a wall while turning, if one is found, 
       * beep, then stop the robot and record the angle it is at in 
       * angleA, the first angle used for angular positioning, then 
       * continue the procedure
       */
      while (true) {
        if (getFilteredData() <= DISTANCE_THRESHOLD){
          Sound.beep();
          Lab4.leftMotor.stop();
          Lab4.rightMotor.stop();
          angleA = odo.getXYT()[2];
          break;
        }
      }


      /* stop checking for distance from wall while the robot turns 90 degrees 
       * to ensure that the sensor does not stop rotation too early
       */
      Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90), true);
      Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90), false);

      //start normal turning clockwise
      Lab4.rightMotor.forward();
      Lab4.leftMotor.backward();

      /* continuously check for a wall while turning, if one is found, beep,
       * then stop the robot and record the angle it is at in angleB,
       * the second angle used for angular positioning
       */
      while (true){
        if (getFilteredData() <= DISTANCE_THRESHOLD){
          Sound.beep();
          Lab4.leftMotor.stop();
          Lab4.rightMotor.stop();
          angleB = odo.getXYT()[2];
          break;
        }
      }

      //calculations for average angle based on formulas given in slides
      if (angleA > angleB){
        angleAvg = 45 - (angleA + angleB)/2;
      }
      else{
        angleAvg =  225 - (angleA + angleB)/2;
      }

      odo.update(0, 0, angleAvg);

      double turnAngle = 0 - odo.getXYT()[2];

      navigation.turnTo(turnAngle);    


    } 

    //Rising Edge Localization
    else if (locVersion == LocalizationVersion.RISING_EDGE){

      /*
       * The robot should turn until it sees the wall, then look for the
       * "rising edges:" the points where it no longer sees the wall.
       * This is very similar to the FALLING_EDGE routine, but the robot
       * will face toward the wall for most of it.
       */

      Lab4.rightMotor.setSpeed(ROTATION_SPEED);
      Lab4.leftMotor.setSpeed(ROTATION_SPEED);


      if(getFilteredData() >= DISTANCE_THRESHOLD){
        //turn the robot clockwise
        Lab4.leftMotor.forward();
        Lab4.rightMotor.backward();

        while (true){
          if(getFilteredData() <= DISTANCE_THRESHOLD){
            Lab4.leftMotor.stop();
            Lab4.rightMotor.stop();
            break;
          }
        }
      }

      /* stop checking for distance from wall while the robot turns 45 degrees 
       * to ensure that the sensor does not stop rotation too early by wrongly detecting another falling edge
       */
      Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), true);
      Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), false);

      //start normal turning clockwise
      Lab4.leftMotor.forward();
      Lab4.rightMotor.backward();

      /* continuously check for a distance farther than the threshold
       * while turning, if one is found, beep, then stop the robot
       * and record the angle it is at in angleA, the first angle used for
       * angular positioning
       */
      while (true){
        if(getFilteredData() > DISTANCE_THRESHOLD){
          Sound.beep();
          Lab4.leftMotor.stop();
          Lab4.rightMotor.stop();
          angleA = odo.getXYT()[2];
          break;
        }
      }

      /* Turn toward the wall a bit to ensure the sensor detects the wall
       * properly and does not accidentally detect it is farther than the
       * threshold
       */
      Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), true);
      Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), false);

      //start normal turning counterclockwise
      Lab4.leftMotor.backward();
      Lab4.rightMotor.forward();

      /* continuously check for a distance farther than the threshold 
       * while turning, if one is found, beep, then stop the robot
       * and record the angle it is at in angleB, the second angle used for
       * angular positioning
       */         
      while (true){
        if(getFilteredData() > DISTANCE_THRESHOLD){
          Sound.beep();
          Lab4.rightMotor.stop();
          Lab4.leftMotor.stop();
          angleB = odo.getXYT()[2];
          break;
        }
      }

      //calculations for average angle based on formulas given in slides
      if(angleA > angleB){
        angleAvg = 225 - (angleA + angleB)/2;
      }
      else{
        angleAvg =  45 - (angleA + angleB)/2;
      }

      odo.update(0, 0, angleAvg);

      double turnAngle = 0 - odo.getXYT()[2];

      navigation.turnTo(turnAngle);    

    }
  }

  /**
   * Helper method that returns the distance of the closest object to the Ultrasonic sensor
   * 
   * @return float representing the distance of the closest object to the Ultrasonic sensor
   */
  public float getFilteredData() {

    usSensor.fetchSample(usData, 0);
    float distance = 100*usData[0];

    return distance;
  }

  //helper methods taken from provided SquareDriver class in Lab2
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

}


