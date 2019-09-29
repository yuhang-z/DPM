package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Display;
import ca.mcgill.ecse211.lab4.UltrasonicLocalizer.LocalizationVersion;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the main class that is run to perform the requirements of lab 4.
 * The main method enables the user to choose between the Falling or Rising edge routines.
 * After Ultrasonic Localization, user input is expected before performing Light Localization.
 * 
 * @author Mohamed Youssef Bouaouina
 * @author Yuhang Zhang
 *
 */
public class Lab4 {

  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port colorPort = LocalEV3.get().getPort("S2");   
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 13.35;
  public static final double L_SENSOR_OFFSET = 13.3; //Distance from light sensor to center of rotation 

  public static void main(String[] args) throws Exception {

    int buttonChoice;

    //Initialize ultrasonic sensor and data buffer
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usValue = usSensor.getMode("Distance");     
    float[] usData = new float[usValue.sampleSize()];

    
    //Initialize light sensor and sample provider
    @SuppressWarnings("resource")
    SensorModes colorSensor = new EV3ColorSensor(colorPort);
    SampleProvider colorValue = colorSensor.getMode("RGB");

    //Initialize Odometer, Display, and Navigation objects  
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Display odometryDisplay = new Display(lcd);
    Navigation navigation = new Navigation(leftMotor, rightMotor, odometer);


    do {
      // clear the display
      lcd.clear();

      // ask the user whether to use Falling or Rising Edge version of US Localization  
      lcd.drawString("Ultrasonic", 0, 0);
      lcd.drawString("Localization", 0, 1);
      lcd.drawString("        |         ", 0, 2);
      lcd.drawString("< Left  | Right > ", 0, 3);
      lcd.drawString("Falling | Rising  ", 0, 4);
      lcd.drawString("   edge | edge    ", 0, 5);
      lcd.drawString("        |         ", 0, 6);

      // waiting for the button to be clicked
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {//Falling edge chosen
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, usSensor, usData, LocalizationVersion.RISING_EDGE, navigation);

      Thread odoThread = new Thread(odometer);
      odoThread.start();

      Thread odometryDisplayThread = new Thread(odometryDisplay);
      odometryDisplayThread.start();          

      usLocalizer.doLocalization();

    }else if (buttonChoice == Button.ID_RIGHT) {//Rising edge chosen
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, usSensor, usData, LocalizationVersion.RISING_EDGE, navigation);

      Thread odoThread = new Thread(odometer);
      odoThread.start();

      Thread odometryDisplayThread = new Thread(odometryDisplay);
      odometryDisplayThread.start();          

      usLocalizer.doLocalization();
    }


    //When done with Ultrasonic Localizaton
    //Wait for button click to continue with Light Localization

    lcd.clear();

    lcd.drawString("To Continue:", 0, 4);
    lcd.drawString("Press any button", 0, 5);

    Button.waitForAnyPress();

    lcd.clear();

    LightLocalizer lLocalizer = new LightLocalizer(odometer, colorValue, navigation);

    lLocalizer.doLocalization();

  }
}
