package ca.mcgill.ecse211.project.localizer;

import ca.mcgill.ecse211.project.controller.Controller;
import ca.mcgill.ecse211.project.controller.Navigation;
import ca.mcgill.ecse211.project.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class controls the second half of localization which uses the light
 * sensor to navigate to the nearest tile intersection. It is intended to be
 * used after ultrasonic localization has completed.
 * 
 * @author Max Musing
 *
 */
public class LightLocalizer {
	private final double LINE_SENSOR_OFFSET;
	private final double TILE_SIZE;
	private final double TRACK;
	private final double WHEEL_RADIUS;

	private final Navigation navigation;
	private final Odometer odometer;
	private final EV3LargeRegulatedMotor leftMotor;
	private final EV3LargeRegulatedMotor rightMotor;
	private final EV3ColorSensor colorSensor;
	private final SensorMode sensorMode;

	public LightLocalizer() {
		this.LINE_SENSOR_OFFSET = Controller.LINE_SENSOR_OFFSET;
		this.TILE_SIZE = Controller.TILE_SIZE;
		this.TRACK = Controller.TRACK;
		this.WHEEL_RADIUS = Controller.WHEEL_RADIUS;

		this.navigation = Controller.navigation;
		this.odometer = Controller.odometer;
		this.leftMotor = Controller.leftMotor;
		this.rightMotor = Controller.rightMotor;

		this.colorSensor = Controller.colorSensorLeft;
		this.sensorMode = colorSensor.getRedMode();
	}

	/**
	 * Updates the robot's odometer from any corner following ultrasonic
	 * localization
	 */
	public void localize() {
		// Calculate the position offset for localization
		double offset = TILE_SIZE - LINE_SENSOR_OFFSET;

		// Align with the back wall
		navigation.moveDistance(-TILE_SIZE / 2, 12);

		// Move to the first line and correct
		navigation.moveDistance(TILE_SIZE * (2 / 3.0), 12);
		navigation.moveToLine();

		// Set the odometer value
		switch (Controller.startingCorner()) {
			case 0:
				odometer.setY(offset);
				odometer.setTheta(90);
				break;

			case 1:
				odometer.setX(12 * TILE_SIZE - offset);
				odometer.setTheta(180);
				break;

			case 2:
				odometer.setY(12 * TILE_SIZE - offset);
				odometer.setTheta(270);
				break;

			case 3:
				odometer.setX(offset);
				odometer.setTheta(0);
				break;
		}

		// Centre on the line
		navigation.moveDistance(LINE_SENSOR_OFFSET, 12);

		// Move to the next line and correct
		navigation.turnAngle(-90, 55);
		navigation.moveDistance(TILE_SIZE / 3, 12);
		navigation.correctOnLine();

		// Set the odometer value, including angle
		switch (Controller.startingCorner()) {
			case 0:
				odometer.setX(offset);
				odometer.setTheta(0);
				break;

			case 1:
				odometer.setY(offset);
				odometer.setTheta(90);
				break;

			case 2:
				odometer.setX(12 * TILE_SIZE - offset);
				odometer.setTheta(180);
				break;

			case 3:
				odometer.setY(12 * TILE_SIZE - offset);
				odometer.setTheta(270);
				break;
		}

		// Beep to signify completed localization
		Sound.beep();
	}
}
