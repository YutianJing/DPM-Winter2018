package ca.mcgill.ecse211.project.controller;

import ca.mcgill.ecse211.project.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to run the search method used to find the flag. Its single
 * public method, search, runs all steps required to find the block, starting
 * from one of the corners of the search region.
 * 
 * @author Max Musing
 *
 */
public class Searcher {
	private final double BUFFER_DISTANCE_ANGLE = Math.PI / 12;
	private SampleProvider us;
	private float[] usData;
	private final Odometer odometer;
	private final Navigation navigation;
	private final EV3LargeRegulatedMotor leftMotor;
	private final EV3LargeRegulatedMotor rightMotor;
	private final NXTRegulatedMotor midMotor;
	private final TextLCD lcd = LocalEV3.get().getTextLCD();
	private boolean targetBlockDetected;

	public Searcher() {
		this.us = Controller.usDistance;
		this.usData = Controller.usData;
		this.odometer = Controller.odometer;
		this.navigation = Controller.navigation;
		this.leftMotor = Controller.leftMotor;
		this.rightMotor = Controller.rightMotor;
		this.midMotor = Controller.midMotor;

		this.targetBlockDetected = false;
	}

	/**
	 * Controls the robot to scan all blocks in the search region and find the
	 * correctly coloured flag. Requires the robot to start at one of the 4
	 * corners of the search region.
	 */
	public void search() {
		navigation.turnTo(90);

		midMotor.setSpeed(150);
		midMotor.rotate(-100, false);

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}

		while (odometer.getXYT()[1] < Controller.URy() * Controller.TILE_SIZE && !targetBlockDetected) {
			if (!blockDetected()) {
				leftMotor.setSpeed(75);
				rightMotor.setSpeed(75);
				leftMotor.forward();
				rightMotor.forward();
			} else {
				Sound.beep();

				// Buffer distance
				double bufferDistance = getDistance() * Math.sin(BUFFER_DISTANCE_ANGLE);
				leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, bufferDistance), true);
				rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, bufferDistance), false);

				// Turn toward the block
				leftMotor.rotate(convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), true);
				rightMotor.rotate(-convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), false);

				// Straighten the usSensor
				midMotor.rotate(100, false);

				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
				}

				// If no block detected, return
				if (blockDetected()) {
					// Drive close to the block
					leftMotor.setSpeed(100);
					rightMotor.setSpeed(100);
					while (getDistance() > 6) {
						leftMotor.forward();
						rightMotor.forward();
					}

					leftMotor.setSpeed(50);
					rightMotor.setSpeed(50);
					while (Controller.display.getColour() == -1) {
						leftMotor.forward();
						rightMotor.forward();
					}

					leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 2), true);
					rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 2), false);

					// Block colour detection
					if (Controller.display.getColour() == Controller.TB()) {
						Sound.beep();
						Sound.beep();
						Sound.beep();
						targetBlockDetected = true;
					} else {
						Sound.beep();
					}

					// Move back to the line
					leftMotor.setSpeed(125);
					rightMotor.setSpeed(125);
					while (odometer.getXYT()[0] > Controller.LLx() * Controller.TILE_SIZE) {
						leftMotor.backward();
						rightMotor.backward();
					}
				}

				// Turn toward the line
				leftMotor.rotate(-convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), true);
				rightMotor.rotate(convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), false);

				// Straighten the usSensor
				midMotor.rotate(-100, false);

				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}

				// Buffer distance
				leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 13), true);
				rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 13), false);
			}
		}

		// Repeat search for next line
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}

		if (targetBlockDetected) {
			navigation.travelTo(Controller.LLx(), Controller.URy());
		}

		// Turn toward the next line
		leftMotor.rotate(convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), false);

		while (odometer.getXYT()[0] < Controller.URx() * Controller.TILE_SIZE && !targetBlockDetected) {
			lcd.clear();
			lcd.drawInt((int) getDistance(), 0, 0);

			if (!blockDetected()) {
				leftMotor.setSpeed(75);
				rightMotor.setSpeed(75);
				leftMotor.forward();
				rightMotor.forward();
			} else {
				Sound.beep();

				// Buffer distance
				double bufferDistance = getDistance() * Math.sin(BUFFER_DISTANCE_ANGLE);
				leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, bufferDistance), true);
				rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, bufferDistance), false);

				// Turn toward the block
				leftMotor.rotate(convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), true);
				rightMotor.rotate(-convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), false);

				// Straighten the usSensor
				midMotor.rotate(90, false);

				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
				}

				// If no block detected, return
				if (blockDetected()) {
					// Drive close to the block
					leftMotor.setSpeed(100);
					rightMotor.setSpeed(100);
					while (getDistance() > 6) {
						leftMotor.forward();
						rightMotor.forward();
					}

					leftMotor.setSpeed(50);
					rightMotor.setSpeed(50);
					while (Controller.display.getColour() == -1) {
						leftMotor.forward();
						rightMotor.forward();
					}

					leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 1.5), true);
					rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 1.5), false);

					// Block colour detection
					if (Controller.display.getColour() == Controller.TB()) {
						Sound.beep();
						targetBlockDetected = true;
					} else {
						Sound.beep();
						Sound.beep();
					}

					// Move back to the line
					leftMotor.setSpeed(125);
					rightMotor.setSpeed(125);
					while (odometer.getXYT()[1] < Controller.URy() * Controller.TILE_SIZE) {
						leftMotor.backward();
						rightMotor.backward();
					}
				}

				// Turn toward the line
				leftMotor.rotate(-convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), true);
				rightMotor.rotate(convertAngle(Controller.WHEEL_RADIUS, Controller.TRACK, 90), false);

				// Straighten the usSensor
				midMotor.rotate(-90, false);

				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}

				// Buffer distance
				leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 13), true);
				rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS, 13), false);
			}
		}

		navigation.travelTo(Controller.URx(), Controller.URy());
	}

	private boolean blockDetected() {
		return getDistance() < Controller.TILE_SIZE * (7.5 - Controller.LLx());
	}

	private double getDistance() {
		us.fetchSample(usData, 0);
		return (usData[0] * 100.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
