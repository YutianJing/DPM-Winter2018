package ca.mcgill.ecse211.project.controller;

import ca.mcgill.ecse211.project.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor. It contains all
 * primary high-level methods to control the robot's movement.
 */
public class Navigation {
	private final int MOVE_SPEED = 12;
	private final int ROTATE_SPEED = 35;
	private final double distanceToCentre;

	private double TILE_SIZE;
	private double TRACK;
	private double WHEEL_RADIUS;

	private final Odometer odometer;
	private final EV3LargeRegulatedMotor leftMotor;
	private final EV3LargeRegulatedMotor rightMotor;

	public Navigation() {
		this.TILE_SIZE = Controller.TILE_SIZE;
		this.TRACK = Controller.TRACK;
		this.WHEEL_RADIUS = Controller.WHEEL_RADIUS;

		this.distanceToCentre = TILE_SIZE / 2 + Controller.LINE_SENSOR_OFFSET;
		this.odometer = Controller.odometer;
		this.leftMotor = Controller.leftMotor;
		this.rightMotor = Controller.rightMotor;
	}

	/**
	 * Resets the primary two motors to a fixed acceleration
	 */
	public void resetMotors() {
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);

		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * Controls the robot to travel to the first obstacle
	 */
	public void travelToFirstObstacle() {
		int obLLx = Controller.isRedTeam() ? Controller.brLLx : Controller.tnLLx;
		int obLLy = Controller.isRedTeam() ? Controller.brLLy : Controller.tnLLy;
		int obURx = Controller.isRedTeam() ? Controller.brURx : Controller.tnURx;
		int obURy = Controller.isRedTeam() ? Controller.brURy : Controller.tnURy;
		boolean obIsVertical = Controller.isRedTeam() ? Controller.bridgeIsVertical() : Controller.tunnelIsVertical();

		travelToObstacle(obLLx, obLLy, obURx, obURy, obIsVertical);
	}

	/**
	 * Controls the robot to travel to the second obstacle
	 */
	public void travelToSecondObstacle() {
		int obLLx = !Controller.isRedTeam() ? Controller.brLLx : Controller.tnLLx;
		int obLLy = !Controller.isRedTeam() ? Controller.brLLy : Controller.tnLLy;
		int obURx = !Controller.isRedTeam() ? Controller.brURx : Controller.tnURx;
		int obURy = !Controller.isRedTeam() ? Controller.brURy : Controller.tnURy;
		boolean obIsVertical = !Controller.isRedTeam() ? Controller.bridgeIsVertical() : Controller.tunnelIsVertical();

		travelToObstacle(obLLx, obLLy, obURx, obURy, obIsVertical);
	}

	/**
	 * Controls the robot to travel to an arbitrary obstacle given its
	 * parameters
	 * 
	 * @param obLLx
	 *            Lower left corner x-coordinate of the obstacle
	 * @param obLLy
	 *            Lower left corner y-coordinate of the obstacle
	 * @param obURx
	 *            Upper right corner x-coordinate of the obstacle
	 * @param obURy
	 *            Upper right corner y-coordinate of the obstacle
	 * @param obIsVertical
	 *            Boolean determining the orientation of the obstacle
	 */
	private void travelToObstacle(int obLLx, int obLLy, int obURx, int obURy, boolean obIsVertical) {
		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double y = xyt[1];

		if (obIsVertical) {
			// Vertical
			if (y <= obLLy * TILE_SIZE) {
				// Bottom
				if (x <= obLLx * TILE_SIZE) {
					// Left
					travelTo(obLLx - 0.5, obLLy - 1);
				} else {
					// Right
					travelTo(obURx + 0.5, obLLy - 1);
				}

				turnTo(90);
			} else {
				// Top
				if (x <= obLLx * TILE_SIZE) {
					// Left
					travelTo(obLLx - 0.5, obURy + 1);
				} else {
					// Right
					travelTo(obURx + 0.5, obURy + 1);
				}

				turnTo(270);
			}

			correctOnLine();
			moveDistance(distanceToCentre, 5);

			if (x <= obLLx * TILE_SIZE) {
				// Left
				turnTo(0);
			} else {
				// Right
				turnTo(180);
			}

			moveDistance(TILE_SIZE / 2, 8);
			correctOnLine();
		} else {
			// Horizontal
			if (x <= obLLx * TILE_SIZE) {
				// Left
				if (y <= obLLy * TILE_SIZE) {
					// Bottom
					travelTo(obLLx - 1, obLLy - 0.5);
				} else {
					// Top
					travelTo(obLLx - 1, obURy + 0.5);
				}

				turnTo(0);
			} else {
				// Right
				if (y <= obLLy * TILE_SIZE) {
					// Bottom
					travelTo(obURx + 1, obLLy - 0.5);
				} else {
					// Top
					travelTo(obURx + 1, obURy + 0.5);
				}

				turnTo(180);
			}

			correctOnLine();
			moveDistance(distanceToCentre, 5);

			if (y <= obLLy * TILE_SIZE) {
				// Bottom
				turnTo(90);
			} else {
				// Top
				turnTo(270);
			}

			moveDistance(TILE_SIZE / 2, 8);
			correctOnLine();
		}
	}

	/**
	 * Controls the robot to travel to the starting corner of the search area
	 */
	public void travelToSearchArea() {
		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double y = xyt[1];

		double toX, toY;

		// Travel to the corner of the search region depending on team 
		if (Controller.isRedTeam()) {
			double sgX = (Controller.sgLLx + Controller.sgURx) / 2;
			double sgY = (Controller.sgLLy + Controller.sgURy) / 2;

			// Get the coordinates of the nearest search region corner
			toX = x < sgX * TILE_SIZE ? Controller.sgLLx : Controller.sgURx;
			toY = y < sgY * TILE_SIZE ? Controller.sgLLy : Controller.sgURy;
		} else {
			double srX = (Controller.srLLx + Controller.srURx) / 2;
			double srY = (Controller.srLLy + Controller.srURy) / 2;

			// Get the coordinates of the nearest search region corner
			toX = x < srX * TILE_SIZE ? Controller.srLLx : Controller.srURx;
			toY = y < srY * TILE_SIZE ? Controller.srLLy : Controller.srURy;
		}

		travelTo(toX, toY);

		// Beep to indicate completion
		Sound.beep();
	}

	/**
	 * Controls the robot to cross the first obstacle
	 */
	public void crossFirstObstacle() {
		// Cross the obstacle depending on team
		if (Controller.isRedTeam()) {
			crossBridge();
		} else {
			crossTunnel();
		}

		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double y = xyt[1];

		// Correct on the next line
		correctOnLine();
		moveDistance(distanceToCentre, 5);

		// Turn towards the next destination
		if (Controller.isRedTeam()) {
			// Tunnel
			if (Controller.tunnelIsVertical()) {
				if (x <= Controller.tnx() * TILE_SIZE) {
					turnTo(0);
				} else {
					turnTo(180);
				}
			} else {
				if (y <= Controller.tny() * TILE_SIZE) {
					turnTo(90);
				} else {
					turnTo(270);
				}
			}
		} else {
			// Bridge
			if (Controller.bridgeIsVertical()) {
				if (x <= Controller.brx() * TILE_SIZE) {
					turnTo(0);
				} else {
					turnTo(180);
				}
			} else {
				if (y <= Controller.bry() * TILE_SIZE) {
					turnTo(90);
				} else {
					turnTo(270);
				}
			}
		}

		// Correct on the next line
		moveDistance(TILE_SIZE / 2, 8);
		correctOnLine();
	}

	/**
	 * Controls the robot to cross the second obstacle
	 */
	public void crossSecondObstacle() {
		if (Controller.isRedTeam()) {
			crossTunnel();
		} else {
			crossBridge();
		}

		// Correct on the next line
		correctOnLine();
		moveDistance(distanceToCentre, 5);

		// Face the general direction of the starting corner and correct on the next line
		if (Controller.isRedTeam()) {
			if (Controller.tunnelIsVertical()) {
				if (Controller.startingCorner() == 0 || Controller.startingCorner() == 3) {
					turnTo(180);
				} else {
					turnTo(0);
				}
			} else {
				if (Controller.startingCorner() <= 1) {
					turnTo(270);
				} else {
					turnTo(90);
				}
			}
		} else {
			if (Controller.bridgeIsVertical()) {
				if (Controller.startingCorner() == 0 || Controller.startingCorner() == 3) {
					turnTo(180);
				} else {
					turnTo(0);
				}
			} else {
				if (Controller.startingCorner() <= 1) {
					turnTo(270);
				} else {
					turnTo(90);
				}
			}
		}

		moveDistance(TILE_SIZE / 2, 8);
		correctOnLine();
	}

	/**
	 * Controls the robot to align with the bridge and cross it to get to the
	 * other area
	 */
	private void crossBridge() {
		double bridgeOffset = 0.25;
		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double y = xyt[1];

		// Align with the track
		moveDistance(bridgeOffset, 5);

		// Turn to face the bridge for crossing
		if (Controller.bridgeIsVertical()) {
			if (y <= Controller.bry() * TILE_SIZE) {
				turnTo(90);
			} else {
				turnTo(270);
			}
		} else {
			if (x <= Controller.brx() * TILE_SIZE) {
				turnTo(0);
			} else {
				turnTo(180);
			}
		}

		// Cross the bridge
		moveDistance((Controller.bridgeLength() + 1.5) * TILE_SIZE, 15);

		// Move to the centre of the tile
		if (Controller.bridgeIsVertical()) {
			if (x <= Controller.brx() * TILE_SIZE) {
				turnTo(0);
			} else {
				turnTo(180);
			}
		} else {
			if (y <= Controller.bry() * TILE_SIZE) {
				turnTo(90);
			} else {
				turnTo(270);
			}
		}

		moveDistance(distanceToCentre - bridgeOffset, 15);

		if (Controller.bridgeIsVertical()) {
			if (y <= Controller.bry() * TILE_SIZE) {
				turnTo(90);
			} else {
				turnTo(270);
			}
		} else {
			if (x <= Controller.brx() * TILE_SIZE) {
				turnTo(0);
			} else {
				turnTo(180);
			}
		}
	}

	/**
	 * Controls the robot to align with the tunnel and cross it to get to the
	 * other area
	 */
	private void crossTunnel() {
		double[] xyt = odometer.getXYT();
		double x = xyt[0];
		double y = xyt[1];

		// Align with the centre of the tunnel
		moveDistance(distanceToCentre, 5);

		// Turn to face the tunnel for crossing
		if (Controller.tunnelIsVertical()) {
			if (y <= Controller.tny() * TILE_SIZE) {
				turnTo(90);
			} else {
				turnTo(270);
			}
		} else {
			if (x <= Controller.tnx() * TILE_SIZE) {
				turnTo(0);
			} else {
				turnTo(180);
			}
		}

		// Straighten in front of the tunnel and cross it
		moveDistance(TILE_SIZE / 3, 15);
		correctOnLine();
		moveDistance((Controller.tunnelLength() + 0.5) * TILE_SIZE, 15);
	}

	/**
	 * Controls the robot to return to the starting position, starting from the
	 * correct area
	 */
	public void travelToStart() {
		switch (Controller.startingCorner()) {
			case 0:
				travelTo(0.5, 0.5);
				break;

			case 1:
				travelTo(11.5, 0.5);
				break;

			case 2:
				travelTo(11.5, 11.5);
				break;

			case 3:
				travelTo(0.5, 11.5);
				break;
		}

		// Move a little extra to ensure the robot is in the corner
		moveDistanceUntracked(TILE_SIZE / 2, 8);
	}

	/**
	 * Controls the robot to travel to the specified map coordinates via the
	 * shortest route and minimal angle
	 * 
	 * @param x
	 *            x-coordinate to travel to
	 * @param y
	 *            y-coordinate to travel to
	 */
	public void travelTo(double x, double y) {
		// Determine the odometer position to travel to in cm
		double destinationX = x * TILE_SIZE;
		double destinationY = y * TILE_SIZE;

		// Calculate the distance and direction to the destination
		double[] xyt = odometer.getXYT();
		double currentX = xyt[0];
		double currentY = xyt[1];
		double distance = this.distance(currentX, currentY, destinationX, destinationY);
		double direction = this.direction(currentX, currentY, destinationX, destinationY);

		// Move to the waypoint
		turnTo(direction);
		moveDistance(distance, MOVE_SPEED);
	}

	/**
	 * Turns the robot to point in a given direction based on the odometer
	 * 
	 * @param theta
	 *            Angle to point towards
	 */
	public void turnTo(double theta) {
		// Calculate speed and direction left to travel
		double[] xyt = odometer.getXYT();
		double currentTheta = xyt[2];
		double angleDifference = angleDifference(currentTheta, theta);

		// Determine direction to turn for minimum angle
		if ((theta - currentTheta + 360) % 360 > 180) {
			turnAngle(-angleDifference, ROTATE_SPEED);
		} else {
			turnAngle(angleDifference, ROTATE_SPEED);
		}
	}

	/**
	 * Moves the robot straight forward a given distance
	 * 
	 * @param distance
	 *            Distance to travel in cm. Can be negative to move backwards
	 * @param speed
	 *            Speed to travel in cm/s
	 */
	public void moveDistance(double distance, double speed) {
		int rotationsPerSecond = convertDistance(WHEEL_RADIUS, speed);
		int rotations = convertDistance(WHEEL_RADIUS, distance);

		leftMotor.setSpeed(rotationsPerSecond);
		rightMotor.setSpeed(rotationsPerSecond);
		leftMotor.rotate(rotations, true);
		rightMotor.rotate(rotations, false);
	}

	/**
	 * Moves the robot straight forward a given distance without tracking via
	 * odometer
	 * 
	 * @param distance
	 *            Distance to travel in cm. Can be negative to move backwards
	 * @param speed
	 *            Speed to travel in cm/s
	 */
	public void moveDistanceUntracked(double distance, double speed) {
		double[] xyt = odometer.getXYT();
		moveDistance(distance, speed);
		odometer.setXYT(xyt[0], xyt[1], xyt[2]);
	}

	/**
	 * Moves the robot straight forward until a line is detected, upon which the
	 * robot straightens out and corrects its odometer
	 */
	public void correctOnLine() {
		double speed = 5;
		int rotationsPerSecond = convertDistance(WHEEL_RADIUS, speed);

		while (true) {
			boolean overBlackLeft = Controller.odometryCorrection.overBlackLeft();
			boolean overBlackRight = Controller.odometryCorrection.overBlackRight();

			if (overBlackLeft == overBlackRight) {
				leftMotor.setSpeed(rotationsPerSecond);
				rightMotor.setSpeed(rotationsPerSecond);
				leftMotor.forward();
				rightMotor.forward();
			} else {
				if (overBlackLeft) {
					while (!Controller.odometryCorrection.overBlackRight()) {
						leftMotor.setSpeed(rotationsPerSecond / 2);
						rightMotor.setSpeed(rotationsPerSecond / 2);
						leftMotor.stop(true);
						rightMotor.forward();
					}
					Controller.odometryCorrection.round();
				} else {
					while (!Controller.odometryCorrection.overBlackLeft()) {
						leftMotor.setSpeed(rotationsPerSecond / 2);
						rightMotor.setSpeed(rotationsPerSecond / 2);
						rightMotor.stop(true);
						leftMotor.forward();
					}
					Controller.odometryCorrection.round();
				}

				leftMotor.stop(true);
				rightMotor.stop(false);
				break;
			}
		}
	}

	/**
	 * Move to the nearest line
	 */
	public void moveToLine() {
		double speed = 5;
		int rotationsPerSecond = convertDistance(WHEEL_RADIUS, speed);

		leftMotor.setSpeed(rotationsPerSecond);
		rightMotor.setSpeed(rotationsPerSecond);

		while (!Controller.odometryCorrection.overBlackLeft()) {
			leftMotor.forward();
			rightMotor.forward();
		}

		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * Turns the robot on the spot by a given degree
	 * 
	 * @param angle
	 *            Angle to turn the robot in deg. Can be negative to turn
	 *            clockwise
	 * @param speed
	 *            Speed to rotate in deg/s
	 */
	public void turnAngle(double angle, double speed) {
		int rotationsPerSecond = convertAngle(WHEEL_RADIUS, TRACK, speed);
		int rotations = convertAngle(WHEEL_RADIUS, TRACK, angle);

		leftMotor.setSpeed(rotationsPerSecond);
		rightMotor.setSpeed(rotationsPerSecond);
		leftMotor.rotate(-rotations, true);
		rightMotor.rotate(rotations, false);
	}

	/**
	 * Calculate the distance between two points
	 * 
	 * @param x1
	 * @param y1
	 * @param x2
	 * @param y2
	 * @return Distance between points
	 */
	private double distance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}

	/**
	 * Calculate the angle between two points
	 * 
	 * @param x1
	 * @param y1
	 * @param x2
	 * @param y2
	 * @return Angle between points
	 */
	private double direction(double x1, double y1, double x2, double y2) {
		double direction = Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
		return (direction + 360) % 360;
	}

	/**
	 * Calculate the absolute difference between two angles
	 * 
	 * @param theta1
	 * @param theta2
	 * @return Delta angle
	 */
	private double angleDifference(double theta1, double theta2) {
		double dTheta = Math.abs(theta2 - theta1);
		return Math.min(dTheta, 360 - dTheta);
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
