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
public class Parameters5 {

	//Simulation Parameters
	public final static int seed = 40462015; //no effect with gui
	public static final long endTime = 5000; //no effect with gui

	//Agent Parameters
	public static final int defaultFuelLevel = 1200;
	public static final int defaultSensorRange = 3;

	//Environment Parameters
	public static final int xDimension = 300; //size in cells
	public static final int yDimension = 25;

	//Object Parameters
	// mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
	public static final double tileMean = 0.8;
	public static final double holeMean = 0.2;
	public static final double obstacleMean = 4.0;
	public static final double tileDev = 0.01f;
	public static final double holeDev = 0.002f;
	public static final double obstacleDev = 0.2f;
	// the life time of each object
	public static final int lifeTime = 200;

    //Agent Best Setup Parameters
    //In-depth documentation available in class HybridPRSTWAgent
    // Fuel management
    public static final double fuelTolerance = 0.85;
    public static final double hardFuelLimit = 300;
    // Agent target selection
    public static final boolean TSPHeuristic = false;
    public static final double objectLifetimeThreshold = 1.0;
    // Agent communication
    public static final int goalAnnounceCount = 1;
    public static final boolean allowAssistance = false;
	public static final int maxAssistZoneDistance = 1;
}

// Parameters from:
// https://www.youtube.com/watch?v=IfYkoQhRpY8
// https://www.youtube.com/watch?v=LE6BPNBjK7U
// https://www.youtube.com/watch?v=ZFWxbptgFj0
// https://www.youtube.com/watch?v=b5hDuWO0u5w
// https://www.youtube.com/watch?v=w-kkdzI5mao