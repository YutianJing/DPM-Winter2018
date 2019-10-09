package ca.mcgill.ecse211.project.localizer;

import ca.mcgill.ecse211.project.controller.Controller;
import ca.mcgill.ecse211.project.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class controls the first half of localization which uses the ultrasonic
 * sensor to determine and correct the robot's initial heading. It can be used
 * in conjunction with light localization to additionally correc the robot's
 * position.
 * 
 * @author Max Musing
 *
 */
public class UltrasonicLocalizer {
	private SampleProvider us;
	private float[] usData;

	private final Odometer odometer;
	private final EV3LargeRegulatedMotor leftMotor;
	private final EV3LargeRegulatedMotor rightMotor;
	private final double TRACK;
	private final double WHEEL_RADIUS;
	private final int WALL_DISTANCE = 50;

	public UltrasonicLocalizer() {
		this.us = Controller.usDistance;
		this.usData = Controller.usData;
		this.odometer = Controller.odometer;
		this.leftMotor = Controller.leftMotor;
		this.rightMotor = Controller.rightMotor;
		this.TRACK = Controller.TRACK;
		this.WHEEL_RADIUS = Controller.WHEEL_RADIUS;
	}

	/**
	 * Start robot localization
	 */
	public void localize() {
		if (getDistance() < WALL_DISTANCE) {
			fallingEdge();
		} else {
			risingEdge();
		}
	}

	/**
	 * Corrects the robot's heading using the falling edge detection method Best
	 * used when facing toward the corner
	 */
	public void fallingEdge() {
		double theta1, theta2;

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		while (getDistance() < WALL_DISTANCE) {
			leftMotor.forward();
			rightMotor.backward();
		}

		theta1 = odometer.getXYT()[2];

		Controller.navigation.turnAngle(150, 55);

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		while (getDistance() < WALL_DISTANCE) {
			leftMotor.backward();
			rightMotor.forward();
		}

		theta2 = odometer.getXYT()[2];

		updateTheta(theta1, theta2);
	}

	/**
	 * Corrects the robot's heading using the rising edge detection method Best
	 * used when facing away from the corner
	 */
	public void risingEdge() {
		double theta1, theta2;

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		while (getDistance() > WALL_DISTANCE) {
			leftMotor.backward();
			rightMotor.forward();
		}

		theta1 = odometer.getXYT()[2];

		Controller.navigation.turnAngle(-160, 55);

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		while (getDistance() > WALL_DISTANCE) {
			leftMotor.forward();
			rightMotor.backward();
		}

		theta2 = odometer.getXYT()[2];

		updateTheta(theta1, theta2);
	}

	/**
	 * Corrects the robot's angle to 0 degrees based on two direction values
	 * 
	 * @param theta1
	 *            Direction of first point (to the left)
	 * @param theta2
	 *            Direction of second point (to the right)
	 */
	private void updateTheta(double theta1, double theta2) {
		leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);

		double thetaRange = (theta2 - theta1 + 360) % 360;
		double theta = thetaRange / 2.0 + 225;

		odometer.setTheta(theta);

		Controller.navigation.turnAngle((450 - theta) % 360, 55);
	}

	private int getDistance() {
		us.fetchSample(usData, 0);
		return (int) (usData[0] * 100.0);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of
	 * each wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
