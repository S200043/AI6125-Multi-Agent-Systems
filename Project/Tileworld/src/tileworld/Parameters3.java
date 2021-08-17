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
public class Parameters3 {

    //Simulation Parameters
    public final static int seed = 4162012; //no effect with gui
    public static final long endTime = 5000; //no effect with gui

    //Agent Parameters
    public static final int defaultFuelLevel = 500;
    public static final int defaultSensorRange = 3;

    //Environment Parameters
    public static final int xDimension = 100; //size in cells
    public static final int yDimension = 100;

    //Object Parameters
    // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    public static final double tileMean = 0.1;
    public static final double holeMean = 0.1;
    public static final double obstacleMean = 0.1;
    public static final double tileDev = 0.025f;
    public static final double holeDev = 0.025f;
    public static final double obstacleDev = 0.025f;
    // the life time of each object
    public static final int lifeTime = 150;

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
    public static final boolean allowAssistance = true;
	public static final int maxAssistZoneDistance = 1;
}
