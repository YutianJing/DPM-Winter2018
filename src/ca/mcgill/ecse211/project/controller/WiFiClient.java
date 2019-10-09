package ca.mcgill.ecse211.project.controller;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * Example class using WifiConnection to communicate with a server and receive data concerning the
 * competition such as the starting corner the robot is placed in.
 * 
 * Keep in mind that this class is an **example** of how to use the WiFi code; you must use the
 * WifiConnection class yourself in your own code as appropriate. In this example, we simply show
 * how to get and process different types of data.
 * 
 * There are two variables you **MUST** set manually before trying to use this code.
 * 
 * 1. SERVER_IP: The IP address of the computer running the server application. This will be your
 * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
 * In that case, set the IP to 192.168.2.3.
 * 
 * 2. TEAM_NUMBER: your project team number
 * 
 * Note: We System.out.println() instead of LCD printing so that full debug output (e.g. the very
 * long string containing the transmission) can be read on the screen OR a remote console such as
 * the EV3Control program via Bluetooth or WiFi. You can disable printing from the WiFi code via
 * ENABLE_DEBUG_WIFI_PRINT (below).
 * 
 * @author Michael Smith, Tharsan Ponnampalam
 *
 */
public class WiFiClient {

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.3";
  private static final int TEAM_NUMBER = 10;

  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;
  
  
  @SuppressWarnings("rawtypes")
  public static void wifi() {

//    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();
      
      Controller.redTeam = ((Long) data.get("RedTeam")).intValue();			//red team number
      Controller.redCorner = ((Long) data.get("RedCorner")).intValue();		//red team starting corner
      Controller.greenTeam = ((Long) data.get("GreenTeam")).intValue();		//green team number
      Controller.greenCorner = ((Long) data.get("GreenCorner")).intValue();	//green team starting corner
      
      Controller.OG = ((Long) data.get("OG")).intValue();						//color of green opponent flag
      Controller.OR = ((Long) data.get("OR")).intValue();						//color of red opponent flag
      
      Controller.redLLx = ((Long) data.get("Red_LL_x")).intValue();			//x of the lower left corner of Red Zone
      Controller.redLLy = ((Long) data.get("Red_LL_y")).intValue();			//y of the lower left corner of Red Zone
      Controller.redURx = ((Long) data.get("Red_UR_x")).intValue();			//x of the upper right corner of Red Zone
      Controller.redURy = ((Long) data.get("Red_UR_y")).intValue();			//y of the upper right corner of Red Zone
      
      Controller.greenLLx = ((Long) data.get("Green_LL_x")).intValue();		//x of the lower left corner of Green Zone
      Controller.greenLLy = ((Long) data.get("Green_LL_y")).intValue();		//y of the lower left corner of Green Zone
      Controller.greenURx = ((Long) data.get("Green_UR_x")).intValue();		//x of the upper right corner of Green Zone
      Controller.greenURy = ((Long) data.get("Green_UR_y")).intValue();		//y of the upper right corner of Green Zone
      
      Controller.tnLLx = ((Long) data.get("TN_LL_x")).intValue();				//x of the lower left corner of the tunnel footprint
      Controller.tnLLy = ((Long) data.get("TN_LL_y")).intValue();				//y of the lower left corner of the tunnel footprint
      Controller.tnURx = ((Long) data.get("TN_UR_x")).intValue();				//x of the upper right corner of the tunnel footprint
      Controller.tnURy = ((Long) data.get("TN_UR_y")).intValue();				//y of the upper right corner of the tunnel footprint
      
      Controller.brLLx = ((Long) data.get("BR_LL_x")).intValue();				//x of the lower left corner of the bridge footprint
      Controller.brLLy = ((Long) data.get("BR_LL_y")).intValue();				//y of the lower left corner of the bridge footprint
      Controller.brURx = ((Long) data.get("BR_UR_x")).intValue();				//x of the upper right corner of the bridge footprint
      Controller.brURy = ((Long) data.get("BR_UR_y")).intValue();				//y of the upper right corner of the bridge footprint
      
      Controller.srLLx = ((Long) data.get("SR_LL_x")).intValue();	//x of the lower left corner of search region in red player zone
      Controller.srLLy = ((Long) data.get("SR_LL_y")).intValue();	//y of the lower left corner of search region in red player zone
      Controller.srURx = ((Long) data.get("SR_UR_x")).intValue();	//x of the upper right corner of search region in red player zone
      Controller.srURy = ((Long) data.get("SR_UR_y")).intValue();	//y of the upper right corner of search region in red player zone
      
      Controller.sgLLx = ((Long) data.get("SG_LL_x")).intValue();	//x of the lower left corner of search region in green player zone
      Controller.sgLLy = ((Long) data.get("SG_LL_y")).intValue();	//y of the lower left corner of search region in green player zone
      Controller.sgURx = ((Long) data.get("SG_UR_x")).intValue();	//x of the upper right corner of search region in green player zone
      Controller.sgURy = ((Long) data.get("SG_UR_y")).intValue();	//y of the upper right corner of search region in green player zone
      
      
/*
      // Example 1: Print out all received data
      System.out.println("Map:\n" + data);

      // Example 2 : Print out specific values
      int redTeam = ((Long) data.get("RedTeam")).intValue();
      System.out.println("Red Team: " + redTeam);

      int og = ((Long) data.get("OG")).intValue();
      System.out.println("Green opponent flag: " + og);

      // Example 3: Compare value
      int tn_ll_x =  ((Long) data.get("TN_LL_x")).intValue();
      if (tn_ll_x < 5) {
        System.out.println("Tunnel LL corner X < 5");
      }
      else {
        System.out.println("Tunnel LL corner X >= 5");
      }
*/
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
/*
    // Wait until user decides to end program
    Button.waitForAnyPress();
*/
  }
}
