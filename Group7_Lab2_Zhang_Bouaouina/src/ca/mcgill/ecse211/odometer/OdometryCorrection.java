package ca.mcgill.ecse211.odometer;
/**
 * Group_7 YuhangZhang YoussefMohamed
 */
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * we used color sensor as we can trigger a distance calculation using the known gap diatance 
 * the main idea was to restart the distance accumulation at the first x,y axis.
 * Under such a case, the robot will return at the original point with negative value. 
 * import color information by RGB mode by the color sensor 
 * we choose the red mode from red, green, and blue as we found red was most reactive 
 * @author YuhangZhang
 *
 */
public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double size = 30.48;
	private Odometer odometer;
	private static final Port colorPort = LocalEV3.get().getPort("S1");
	SensorModes colorSensor = new EV3ColorSensor(colorPort);
	SampleProvider colorSample = colorSensor.getMode("RGB");
	float[] colorData = new float[colorSample.sampleSize()];
	/**
	 * to determine the position, we set two counters at different positions 
	 * that we can use x_counter*size  to determine the distance the robot has gone 
	 */
	int x_counter = 0;
	int y_counter = 0;
	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}
	/**
	 * to create a static value outside the main function 
	 * startcolorID will store the color intensity at the broad that it was initially put at
	 */
	private static double startcolorID; //create a static variable 

	/**
	 * The odometer correction runs in this method
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)

	public void run() {
		long correctionStart, correctionEnd;
		colorSample.fetchSample(colorData, 0);
		
		for(int i = 1;i<=100;i++) {
			startcolorID += colorData[0]*1000; //scaled by 1000 for accuracy
		}
		startcolorID /= 100; //scaled by 1000 for accuracy
		

		while (true) {
			correctionStart = System.currentTimeMillis();

			colorSample.fetchSample(colorData, 0);
			/**
			 * colorid is a refreshing float that would store the color intensity from time to time 
			 * and if colorid is away from startcolorID, that means the robot is crossing a different color 
			 */
			double colorid = colorData[0]*1000; //scaled by 1000 for accuracy

			LCD.drawString("Intensity "+ colorid, 0, 6 ); // display initial information on screen 
			LCD.drawString("firstcolor "+ startcolorID, 0, 5 ); // display information on screen 

			if (Math.abs(startcolorID - colorid) > 25) {

				Sound.beep();
				//LCD.drawString("Intensity " + colorData[0]*1000, 0, 5 );

				//check the current orientation of traveling and save it to current_angle
				double current_angle = odometer.getXYT()[2];
				/**
				 * when the robot reach the first line; reset the coordinate to zero
				 * when the robot is in the y forward direction, the correction distance should be 
				 * the number of squares* size of a square
				 * 
				 */
				if ((current_angle >= 0.00 && current_angle <= 20) || (current_angle >= 340 && current_angle <= 360)) {
					odometer.setY(y_counter * size);
					y_counter++;
				}
				/**
				 *first time traveling in x direction, the x should be reset to zero 
				 * when the robot is in the x towards right direction, the correction distance should be 
				 * the number of squares* size of a square
				 * 
				 */
				else if ((current_angle >= 70 && current_angle <= 110)) {
					odometer.setX(x_counter * size);
					x_counter++;
				}
				// when the robot is in the y direction backward, the y value will decrease (finally to a negative value)
				else if (current_angle >= 160 && current_angle <= 200) {
					y_counter--;
					odometer.setY(y_counter * size);
				}
				// when the robot is in the x left direction, the x will decrease (finally to a negative value)
				else if (current_angle >= 250 && current_angle <= 290) {
					x_counter--;
					odometer.setX(x_counter * size);
				}
			}



			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}

