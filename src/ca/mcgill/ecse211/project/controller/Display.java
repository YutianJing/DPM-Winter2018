package ca.mcgill.ecse211.project.controller;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.odometer.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta)
 */
public class Display implements Runnable {
	private final double TILE_SIZE;
	private TextLCD lcd;
	private Odometer odometer;
	private double[] position;
	private final long DISPLAY_PERIOD = 25;
	private long timeout = Long.MAX_VALUE;
	private final EV3ColorSensor colorSensor;
	private final SensorMode sensorMode;

	public Display() {
		this.lcd = Controller.lcd;
		this.colorSensor = Controller.colorSensorMiddle;
		this.odometer = Controller.odometer;
		this.sensorMode = colorSensor.getRGBMode();
		this.TILE_SIZE = Controller.TILE_SIZE;
	}

	public void run() {

		lcd.clear();

		long updateStart, updateEnd;
		long tStart = System.currentTimeMillis();

		do {
			updateStart = System.currentTimeMillis();

			// Retrieve x, y and Theta information
			position = odometer.getXYT();

			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0] / (double) TILE_SIZE), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1] / (double) TILE_SIZE), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

			// This ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} while ((updateEnd - tStart) <= timeout);

	}
	
	/**
	 * Get the colour detected by the light sensor
	 * @return Detected colour (1-4 correspond to Red, Blue, Yellow, White)
	 */
	public int getColour() {
		float[] colour = new float[sensorMode.sampleSize()];

		// Sense RGB light values
		sensorMode.fetchSample(colour, 0);
		float red = colour[0];
		float green = colour[1];
		float blue = colour[2];
		
		if (red < 0.02 && green < 0.02) {
			return -1; // No block detected
		} else if (red > 0.1 && blue < 0.02) {
			return 1; // Red
		} else if (red < 0.06) {
			return 2; // Blue
		} else if (red > 0.25 && blue < 0.04) {
			return 3; // Yellow
		} else if (red > 0.25 && blue < 0.3) {
			return 4; // White
		} else {
			return -1; // No block detected
		}
	}
}
