package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

/**
 * This class provides methods to perform light localization. 
 * 
 * @author Mohamed Youssef Bouaouina
 * @author Yuhang Zhang
 */
public class LightLocalizer {

  private SampleProvider colorSensor;
  private static double[] angles;
  public static double[] obtainAngle;
  public static final int ROTATION_SPEED = 75;
  public static final int FORWARD_SPEED = 100;
  public static double excessangle;

  private Odometer odometer;
  private Navigation navigation;

  /**
   * Constructor that provides a LightLocalizer object
   * 
   * @param odo : Odometer object used for determining coordinates of the robot
   * @param colorSensor : Sample Provider of the color sensor
   * @param navigation : Navigation object that provides navigation methods
   */
  public LightLocalizer(Odometer odo, SampleProvider colorSensor, Navigation navigation){
    this.odometer = odo;
    this.colorSensor = colorSensor;
    this.navigation = navigation;
    angles = new double[4];
    obtainAngle= new double[4];
  }


  /*This method does the following:
   * 1) rotates the robot 45 degrees and go to an optimal location
   * 2) rotates the robot in a circle to get angles for all the lines
   * 3) fetches theta values to an array and uses them to calculate the distance to 0,0
   * 4) navigates the robot to 0,0 and turn to 0 degrees
   */
  public void doLocalization(){

    int angle_index = 0;
    odometer.setTheta(0.0);

    rotateClockwise(45);   // rotating robot 45 degress and moving till it sees a line

    /*
     * Used to implement a differential filter:
     * we start by sampling the color intensity of the middle of the tile to be able 
     * to compare to the color intensity of the lines
     */
    float[] startColorId = new float[1];
    colorSensor.fetchSample(startColorId, 0);
    LCD.drawString("ID " + startColorId[0], 0, 6);

    float[] sample = new float[1];

    while (true){
      move(FORWARD_SPEED);
      colorSensor.fetchSample(sample, 0);      
      LCD.drawString("Sample " + sample[0], 0, 7);

      if (Math.abs(startColorId[0] - sample[0]) >= 4.0 ) {
        Sound.beep();
        break;
      }
    }


    moveBackByRobotSize();    // moving the robot back by the size of robot to get to optimal location

    stopMotors();

    wait(1000);

    boolean running = true;

    rotateClockwise();     // rotating the robot in a circle to get angle values


    while(running){

      colorSensor.fetchSample(sample, 0);
      LCD.drawString("Sample " + sample[0], 0, 7);

      if (Math.abs(startColorId[0] - sample[0]) >= 4.0){ // dark line detected
        Sound.beep();
        angles[angle_index]= odometer.getXYT()[2];    // fetching angle values to the robot
        angle_index = angle_index+1;
        if(angle_index == 4){
          stopMotors();
          wait(2000);
          excessangle= odometer.getXYT()[2];
          running = false;
          break;
        }
        wait(400);

      }

    } //end of while running loop

    // calculating distance to 0,0 and fetching theta and distance value to navigation lab
    double ytheta= angles[1]-angles[3]; 
    double xtheta= angles[0]-angles[2];
    double extracorrection= excessangle - angles[3];
    double Xo= position(xtheta);
    double Yo= position(ytheta);
    // double CorrectTheta = 90 + (ytheta/2) - (-ytheta-180);
    double thetaO= (angles[1]-angles[3])/2 -angles[2]+ extracorrection; 
    double correcttheta = odometer.getXYT()[2]+ thetaO ;
    double thetaFinal =angleCorrection(correcttheta);
    // setting values in odometer
    odometer.setXYT(Xo, Yo, thetaFinal);
    navigation.travelTo(0, 0);
    turnTo(-odometer.getXYT()[2]);
  }


  /**
   * Helper method that sends the robot in a rotating motion
   */
  private static void rotateClockwise(){ // tells the robot to keep rotating
    Lab4.leftMotor.setSpeed(ROTATION_SPEED);
    Lab4.rightMotor.setSpeed(ROTATION_SPEED);
    Lab4.leftMotor.forward();
    Lab4.rightMotor.backward();
  }

  /**
   * Helper method that turns the robot clockwise
   * 
   * @param theta : number of degrees to turn
   */
  private static void rotateClockwise(double theta){ // tells the robot to rotate by a certain theta
    Lab4.leftMotor.setSpeed(ROTATION_SPEED);
    Lab4.rightMotor.setSpeed(ROTATION_SPEED);
    Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
    Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
  }

  /**
   * Helper method that stops the motor of the robot
   */
  private void stopMotors(){  // tells robot to stop
    Lab4.leftMotor.stop();
    Lab4.rightMotor.stop();
  }

  /**
   * Helper method that drives the motor forward
   * @param Speed : int representing the travel speed
   */
  private static void move(int Speed) {  // tells robot to move
    Lab4.leftMotor.setSpeed(Speed);
    Lab4.rightMotor.setSpeed(Speed);
    Lab4.leftMotor.forward();
    Lab4.rightMotor.forward();
  }

  /**
   * Helper method that turns the robot with the specified angle
   * 
   * @param angle : double representing the angle to turn 
   */
  public void turnTo(double angle) {

    Lab4.leftMotor.setSpeed(100);
    Lab4.rightMotor.setSpeed(100);

    Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle), true);
    Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle), false);

  }

  /**
   * Helper method that drives the robot backwards by a distance equal to its size
   */
  private static void moveBackByRobotSize(){  // tells robot to move by robot size
    Lab4.leftMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, Lab4.L_SENSOR_OFFSET), true); 
    Lab4.rightMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, Lab4.L_SENSOR_OFFSET), false);
  }

  /**
   * Helper method that makes the Thread sleep
   * 
   * @param time : int representing the number of milliseconds for which the Thread sleeps
   */
  private static void wait(int time){ //tells robot to wait
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  //helper methods taken from provided SquareDriver class in Lab2
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);

  }

  //helper functions

  private double position(double angle) {     // method to find the position
    double position = -Lab4.L_SENSOR_OFFSET *Math.cos((Math.toRadians(angle)/2));
    return position;

  }
  private static double angleCorrection(double angle){  // corrects the theta values
    if (angle > 180) {
      return angle - 360;
    } else if (angle < -180) {
      return angle + 360;
    } else {
      return angle;
    }
  }

}

