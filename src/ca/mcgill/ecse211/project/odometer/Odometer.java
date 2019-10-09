package ca.mcgill.ecse211.project.odometer;

import ca.mcgill.ecse211.project.controller.Controller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class controls the odometer which keeps track of the robot's position.
 * It uses the tachometers of the motors to measure the distance travelled over
 * time.
 * 
 * @author Max Musing
 *
 */
public class Odometer extends OdometerData implements Runnable {
	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RADIUS;

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors
	 * and variables once.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 */
	public Odometer() {
		this.TRACK = Controller.TRACK;
		this.WHEEL_RADIUS = Controller.WHEEL_RADIUS;

		this.leftMotor = Controller.leftMotor;
		this.rightMotor = Controller.rightMotor;

		// Reset the values of x, y and t to initial conditions
		this.setXYT(0, 0, 90);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
	}

	/**
	 * This method is where the logic for the odometer runs in a thread.
	 */
	public void run() {
		long updateStart, updateEnd;

		int leftMotorTachoCountPrev = 0;
		int rightMotorTachoCountPrev = 0;

		while (true) {
			updateStart = System.currentTimeMillis();

			// Get the number of rotations of each motor
			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// Calculate rotations in past cycle
			int dLeftMotorTachoCount = leftMotorTachoCount - leftMotorTachoCountPrev;
			int dRightMotorTachoCount = rightMotorTachoCount - rightMotorTachoCountPrev;

			// Calculate new robot position based on tachometer counts

			// Get current theta (degrees)
			double[] xyt = this.getXYT();
			double theta = xyt[2];

			// Calculate distance traveled by the wheels (cm)
			double distanceLeft = WHEEL_RADIUS * Math.toRadians(dLeftMotorTachoCount);
			double distanceRight = WHEEL_RADIUS * Math.toRadians(dRightMotorTachoCount);

			// Calculate delta theta (degrees)
			double distanceDifference = distanceRight - distanceLeft;
			double dtheta = Math.toDegrees(distanceDifference / TRACK);

			// Calculate delta x and y (cm)
			double displacement = (distanceLeft + distanceRight) / 2.0;
			double dx = displacement * Math.cos(Math.toRadians(theta + dtheta));
			double dy = displacement * Math.sin(Math.toRadians(theta + dtheta));

			// Update odometer values with new calculated values
			this.update(dx, dy, dtheta);

			// Set previous tacho counts
			leftMotorTachoCountPrev = leftMotorTachoCount;
			rightMotorTachoCountPrev = rightMotorTachoCount;

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}
