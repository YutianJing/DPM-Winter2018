package ca.mcgill.ecse211.project.odometer;

import ca.mcgill.ecse211.project.controller.Controller;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class contains high-level functions used to correct the odometer's
 * values. It is intended to be used during localization and correction using
 * line-detection.
 * 
 * @author Max Musing
 *
 */
public class OdometryCorrection {
	private static final double TILE_SIZE = 30.48;
	private static final double SENSOR_OFFSET = 2.5;
	private Odometer odometer;

	private final EV3ColorSensor colorSensorLeft;
	private final EV3ColorSensor colorSensorRight;
	private final SensorMode sensorModeLeft;
	private final SensorMode sensorModeRight;

	/**
	 * This is the default class constructor. An existing instance of the
	 * odometer is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() {
		this.odometer = Controller.odometer;
		this.colorSensorLeft = Controller.colorSensorLeft;
		this.colorSensorRight = Controller.colorSensorRight;
		this.sensorModeLeft = colorSensorLeft.getRedMode();
		this.sensorModeRight = colorSensorRight.getRedMode();
	}

	public void round() {
		roundX();
		roundY();
		roundTheta();
	}

	/**
	 * Round the odometer's x position to the nearest tile size
	 */
	private void roundX() {
		// Get robot position
		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double theta = xyt[2];

		// Determine general direction the robot is facing
		boolean movingRight = theta < 45 || theta > 335;
		boolean movingLeft = Math.abs(theta - 180) < 45;
		boolean movingHorizontally = movingRight || movingLeft;

		if (movingHorizontally) {
			// Account for offset depending on direction
			double offset = movingRight ? SENSOR_OFFSET : -SENSOR_OFFSET;

			// Round x position to nearest tile length plus offset
			double roundedTheta = Math.round(x / TILE_SIZE) * TILE_SIZE + offset;
			odometer.setX(roundedTheta);
		}
	}

	/**
	 * Round the odometer's y position to the nearest tile size
	 */
	private void roundY() {
		// Get robot position
		double[] xyt = odometer.getXYT();
		double y = xyt[1];
		double theta = xyt[2];

		// Determine general direction the robot is facing
		boolean movingUp = Math.abs(theta - 90) < 45;
		boolean movingDown = Math.abs(theta - 270) < 45;
		boolean movingVertically = movingUp || movingDown;

		if (movingVertically) {
			// Account for offset depending on direction
			double offset = movingDown ? SENSOR_OFFSET : -SENSOR_OFFSET;

			// Round y position to nearest tile length plus offset
			double roundedY = Math.round(y / TILE_SIZE) * TILE_SIZE + offset;
			odometer.setY(roundedY);
		}
	}

	/**
	 * Round the odometer's theta to the nearest 90 degrees
	 */
	private void roundTheta() {
		// Get robot position
		double[] xyt = odometer.getXYT();
		double theta = xyt[2];

		double roundedTheta = Math.round(theta / 90.0) * 90;
		odometer.setTheta(roundedTheta);
	}

	/**
	 * @return Boolean whether the left sensor is over a black line
	 */
	public boolean overBlackLeft() {
		return overBlack(sensorModeLeft);
	}

	/**
	 * @return Boolean whether the right sensor is over a black line
	 */
	public boolean overBlackRight() {
		return overBlack(sensorModeRight);
	}

	/**
	 * @return Boolean whether the selected sensor is over a black line
	 */
	private boolean overBlack(SensorMode sensorMode) {
		final float brightnessTrigger = 0.52f;
		float[] colour = new float[sensorMode.sampleSize()];

		// Sense red light value
		sensorMode.fetchSample(colour, 0);
		float brightness = colour[0];

		return brightness < brightnessTrigger;
	}
}
