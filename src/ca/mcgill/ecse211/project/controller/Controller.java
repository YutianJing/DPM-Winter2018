package ca.mcgill.ecse211.project.controller;

import ca.mcgill.ecse211.project.localizer.*;
import ca.mcgill.ecse211.project.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is used to start all threads, including the Controller thread
 * which controls all robot behaviour. It contains high-level functions for
 * accessing the game parameters, singleton classes, and other constants.
 * 
 * @author Max Musing
 *
 */
public class Controller {

	// Starting player corners
	public static int redTeam;
	public static int redCorner;
	public static int greenTeam;
	public static int greenCorner;

	// Flag colours (Red, Blue, Yellow, White)
	public static int OG; // 1-4
	public static int OR; // 1-4

	// Red area (2x2 to 10x10)
	public static int redLLx; // 0-12
	public static int redLLy; // 0-12
	public static int redURx; // 0-12
	public static int redURy; // 0-12

	// Green area (2x2 to 10x10)
	public static int greenLLx; // 0-12
	public static int greenLLy; // 0-12
	public static int greenURx; // 0-12
	public static int greenURy; // 0-12

	// Tunnel position (1x2 or 2x1)
	public static int tnLLx; // 0-12
	public static int tnLLy; // 0-12
	public static int tnURx; // 0-12
	public static int tnURy; // 0-12

	// Bridge position (1x2 or 2x1)
	public static int brLLx; // 0-12
	public static int brLLy; // 0-12
	public static int brURx; // 0-12
	public static int brURy; // 0-12

	// Red search region (2x2 to 10x10)
	public static int srLLx; // 0-12
	public static int srLLy; // 0-12
	public static int srURx; // 0-12
	public static int srURy; // 0-12

	// Green search region (2x2 to 10x10)
	public static int sgLLx; // 0-12
	public static int sgLLy; // 0-12
	public static int sgURx; // 0-12
	public static int sgURy; // 0-12

	public static final double LINE_SENSOR_OFFSET = -11.8;
	public static final double TILE_SIZE = 30.48;
	public static final double TRACK = 15.0;
	public static final double WHEEL_RADIUS = 2.2;

	// Motors
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final NXTRegulatedMotor midMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("C"));

	// Sensors
	public static final EV3ColorSensor colorSensorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static final SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	public static final EV3ColorSensor colorSensorMiddle = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	public static final EV3ColorSensor colorSensorRight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	public static final SampleProvider usDistance = usSensor.getMode("Distance");
	public static final float[] usData = new float[usDistance.sampleSize()];
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Components
	public static final Odometer odometer = new Odometer();
	public static final OdometryCorrection odometryCorrection = new OdometryCorrection();
	public static final Navigation navigation = new Navigation();
	public static final UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
	public static final LightLocalizer lightLocalizer = new LightLocalizer();
	public static final Searcher searcher = new Searcher();
	public static final Display display = new Display();

	public static void main(String[] args) throws InterruptedException {
		// Fetch game parameters
		redTeam = -1;
		WiFiClient.wifi();

		// Wait until data is received over WiFi before starting
		while (redTeam == -1) {
		}

		Thread odoDisplayThread = new Thread(display);
		odoDisplayThread.start();

		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// Run the main thread
		Thread main = new Thread() {
			public void run() {
				navigation.resetMotors();

				usLocalizer.localize();
				lightLocalizer.localize();

				navigation.travelToFirstObstacle();
				navigation.crossFirstObstacle();

				navigation.travelToSearchArea();
				searcher.search();

				navigation.travelToSecondObstacle();
				navigation.crossSecondObstacle();

				navigation.travelToStart();

			}
		};
		main.start();

		// Wait for button press to end the program
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		}

		System.exit(0);
	}

	/**
	 * Determine which team the player is on
	 * 
	 * @return True if red, false is green
	 */
	public static boolean isRedTeam() {
		return redTeam == 10;
	}

	/**
	 * Get the starting corner for the player
	 * 
	 * @return Starting corner from 0-3
	 */
	public static int startingCorner() {
		return isRedTeam() ? redCorner : greenCorner;
	}

	/**
	 * Get the orientation of the tunnel
	 * 
	 * @return True if vertical, false if horizontal
	 */
	public static boolean tunnelIsVertical() {
		return tnLLy == redURy || tnLLy == greenURy;
	}

	/**
	 * Get the orientation of the bridge
	 * 
	 * @return True if vertical, false if horizontal
	 */
	public static boolean bridgeIsVertical() {
		return brLLy == redURy || brLLy == greenURy;
	}

	/**
	 * Get the length of the tunnel taking into account orientation
	 * 
	 * @return Length of tunnel in tiles
	 */
	public static int tunnelLength() {
		return tunnelIsVertical() ? tnURy - tnLLy : tnURx - tnLLx;
	}

	/**
	 * Get the length of the bridge taking into account orientation
	 * 
	 * @return Length of bridge in tiles
	 */
	public static int bridgeLength() {
		return bridgeIsVertical() ? brURy - brLLy : brURx - brLLx;
	}

	/**
	 * Get the x coordinate of the centre of the bridge
	 * 
	 * @return x coordinate of the bridge
	 */
	public static double brx() {
		return (brLLx + brURx) / 2.0;
	}

	/**
	 * Get the y coordinate of the centre of the bridge
	 * 
	 * @return y coordinate of the bridge
	 */
	public static double bry() {
		return (brLLy + brURy) / 2.0;
	}

	/**
	 * Get the x coordinate of the centre of the tunnel
	 * 
	 * @return x coordinate of the tunnel
	 */
	public static double tnx() {
		return (tnLLx + tnURx) / 2.0;
	}

	/**
	 * Get the y coordinate of the centre of the tunnel
	 * 
	 * @return y coordinate of the tunnel
	 */
	public static double tny() {
		return (tnLLy + tnURy) / 2.0;
	}

	/**
	 * Get the lower left x coordinate for the player's search region
	 * 
	 * @return Lower left x coordinate
	 */
	public static int LLx() {
		return isRedTeam() ? sgLLx : srLLx;
	}

	/**
	 * Get the lower left y coordinate for the player's search region
	 * 
	 * @return Lower left y coordinate
	 */
	public static int LLy() {
		return isRedTeam() ? sgLLy : srLLy;
	}

	/**
	 * Get the upper right x coordinate for the player's search region
	 * 
	 * @return Upper right x coordinate
	 */
	public static int URx() {
		return isRedTeam() ? sgURx : srURx;
	}

	/**
	 * Get the upper right y coordinate for the player's search region
	 * 
	 * @return Upper right y coordinate
	 */
	public static int URy() {
		return isRedTeam() ? sgURx : srURx;
	}

	/**
	 * Get the target block colour for the player
	 * 
	 * @return Target block colour (1-4 correspond to Red, Blue, Yellow, White)
	 */
	public static int TB() {
		return isRedTeam() ? OG : OR;
	}
}
