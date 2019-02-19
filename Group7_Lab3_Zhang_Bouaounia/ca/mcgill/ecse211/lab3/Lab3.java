package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Display;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is run on the robot to perform the requirements for lab 3.
 * Enables the user to choose the map of waypoints to traverse and the type of navigation. 
 * 
 * @modified Mohamed Youssef Bouaouina
 * @modified YuHang Zhang
 *
 */
public class Lab3 {

  /*
   * Maps used for the demo
   */
  public static final int[][] MAP_ONE = {{0,2} , {1,1} , {2,2} , {2,1} , {1,0}};
  public static final int[][] MAP_TWO = {{1,1} , {0,2} , {2,2} , {2,1} , {1,0}};
  public static final int[][] MAP_THREE = {{1,0} , {2,1} , {2,2} , {0,2} , {1,1}};
  public static final int[][] MAP_FOUR = {{0,1} , {1,2} , {1,0} , {2,1} , {2,2}}; 

  //Map used for lab report tests
  public static final int[][] TEST_MAP = {{2,1} , {1,1} , {1,2} , {2,0}}; 

  /* constants */
  // connect the ports with the left motor, the right motor, the ultra sonic
  // sensor
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 11.55;

  public static void main(String[] args) throws Exception {

    //to be selected on the day of the demo by the TA
    int[][] waypoints = MAP_ONE; 


    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    int buttonChoice;

    // instantiating Odometer, Avoidance, Navigation, and Display objects 
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Avoidance avoidance = new Avoidance(leftMotor, rightMotor);
    Navigation navigation = new Navigation(leftMotor, rightMotor, waypoints, odometer, usDistance, usData, avoidance);
    Display odometryDisplay = new Display(lcd);

    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("   No  | With   ", 0, 1);
      lcd.drawString(" avoid | avoid  ", 0, 2);
      lcd.drawString("       |        ", 0, 3);
      // waiting for the button to be clicked
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {

      Thread odoThread = new Thread(odometer);
      odoThread.start();

      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      Thread navigationthread = new Thread(navigation);
      navigationthread.start();

    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);

  }

}
