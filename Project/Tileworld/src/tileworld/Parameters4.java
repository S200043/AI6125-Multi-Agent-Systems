package tileworld;

/**
 * Parameters
 *
 * @author michaellees
 * Created: Apr 21, 2010
 *
 * Copyright michaellees
 *
 * Description:
 *
 * Class used to store global simulation parameters.
 * Environment related parameters are still in the TWEnvironment class.
 *
 */
public class Parameters4 {

	//Simulation Parameters
	public final static int seed = 9042014; //09042014 //no effect with gui
	public static final long endTime = 5000; //no effect with gui

	//Agent Parameters
	public static final int defaultFuelLevel = 1000;
	public static final int defaultSensorRange = 3;

	//Environment Parameters
	public static final int xDimension = 150; //size in cells
	public static final int yDimension = 40;

	//Object Parameters
	// mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
	public static final double tileMean = 0.02;
	public static final double holeMean = 0.2;
	public static final double obstacleMean = 0.5;
	public static final double tileDev = 0.001f;
	public static final double holeDev = 0.01f;
	public static final double obstacleDev = 0.1f;
	// the life time of each object
	public static final int lifeTime = 120;

    //Agent Best Setup Parameters
    //In-depth documentation available in class HybridPRSTWAgent
    // Fuel management
    public static final double fuelTolerance = 0.95;
    public static final double hardFuelLimit = 100;
    // Agent target selection
    public static final boolean TSPHeuristic = true;
    public static final double objectLifetimeThreshold = 1.0;
    // Agent communication
    public static final int goalAnnounceCount = 1;
    public static final boolean allowAssistance = false;
	public static final int maxAssistZoneDistance = 1;
}

// Parameters from:
// https://www.youtube.com/watch?v=A4eSjiCcLok
// https://www.youtube.com/watch?v=q-RA0Y9-j0g
// https://www.youtube.com/watch?v=01cPSrwCUAM
// https://www.youtube.com/watch?v=FcIl_6ezG-w
// https://www.youtube.com/watch?v=GfHJWebig1g
// https://www.youtube.com/watch?v=dS7GoNAD5pw