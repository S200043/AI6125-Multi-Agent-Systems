package tileworld.agent;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.PriorityQueue;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.DefaultTWPlanner;

public class HybridPRSTWAgent extends TWAgent {
	//*********************************
	//***** Adjustable Parameters *****
	//*********************************
	/** When comparing remaining fuel to distance to fuel station, how much buffer to account for
	 *  Higher tolerance means agent leaves little buffer fuel to account for obstacles appearing
	 *  Lower tolerance means agent leaves plenty of buffer fuel and may tend to refuel more
	 */
	private double fuelTolerance = Parameters.fuelTolerance;

	/** Hard fuel limit before needing to refuel
	 */
	private double hardFuelLimit = Parameters.hardFuelLimit;

	/** Modifies the heuristic used for prioritizing the list of possible goals. 
	 */
	private boolean TSPHeuristic = Parameters.TSPHeuristic;

	/** Lifetime threshold used for determining whether a object at risk of decay should be pursued,
	 *  taking into account both its estimated remaining lifetime and its distance from agent.
	 *  Estimated remaining time left for a memorized object is based on its memory time stamp.
	 */
	private double objectLifetimeThreshold = Parameters.objectLifetimeThreshold;

	/** Maximum number of goals in queue to announce.
	 *  Announcing goals prevent goal collisions.
	 *  However, reserving too many goals can lead to sub-optimal division of goals between agents.
	 *  Reserving too little can result in collision between an assisting agent's goal and this agent's next immediate goal,
	 *  thus wasting the assisting agent's resources.
	 */
	private int goalAnnounceCount = Parameters.goalAnnounceCount;

	private boolean allowAssistance = Parameters.allowAssistance;

	/** Furthest zone agent can move to assist, specified in terms of number of zones
	 */
	private int maxAssistZoneDistance = Parameters.maxAssistZoneDistance;
	//*********************************
	//*********************************
	//*********************************

	enum Mode {
		EXPLORE, COLLECT, FILL, REFUEL, ASSIST_COLLECT, ASSIST_FILL, REACT_COLLECT, REACT_FILL, WAIT
	}

	private String name;
	private int agentIdx;
	private DefaultTWPlanner planner;
	private Mode mode;
	private boolean zoneByRows;
	private Integer[] agentZones;
	private Int2D[] bounds; // Bounds position clock-wise from top-left
	private Int2D[] anchors;
	private TWAgentDecayMemory decayMemory;

	PriorityQueue<TWEntity> tilesInZone;
	PriorityQueue<TWEntity> holesInZone;
	LinkedList<TWEntity> possibleTileGoals;
	LinkedList<TWEntity> possibleHoleGoals;
	TWEntity[] closestTile;
	TWEntity[] closestHole;

	public HybridPRSTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
		super(xpos, ypos, env, fuelLevel);
		this.name = name;
		this.agentIdx = Character.getNumericValue(name.charAt(name.length() - 1)) - 1;
		this.planner = new DefaultTWPlanner(this);
		this.memory = new TWAgentDecayMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
		this.decayMemory = (TWAgentDecayMemory) this.memory;
		this.bounds = new Int2D[4];
		this.tilesInZone = new PriorityQueue<TWEntity>();
		this.holesInZone = new PriorityQueue<TWEntity>();
		this.possibleTileGoals = new LinkedList<TWEntity>();
		this.possibleHoleGoals = new LinkedList<TWEntity>();
	}

	@Override
	public void communicate() {
		Message message = new TypedMessage(name, "ALL", "MAP", new Object[] { decayMemory.getAgentPercept(), new Int2D(x, y) });
		this.getEnvironment().receiveMessage(message); // this will send the message to the broadcast channel of the environment

		// Broadcast surplus tiles and holes for auction
		if (bounds[0] != null) {
			tilesInZone = decayMemory.getNearbyObjectsWithinBounds(bounds, new TWTile().getClass());
			holesInZone = decayMemory.getNearbyObjectsWithinBounds(bounds, new TWHole().getClass());

			possibleTileGoals.clear();
			possibleHoleGoals.clear();

			LinkedList<TWEntity> goals = new LinkedList<TWEntity>();
			LinkedList<TWEntity> auctionTiles = new LinkedList<TWEntity>();
			LinkedList<TWEntity> auctionHoles = new LinkedList<TWEntity>();
			// Cull PriorityQueues for objects too far away to reach in time and auction them to other nearby agents
			while (!tilesInZone.isEmpty()) {
				TWEntity tile = tilesInZone.poll();
				double distToTile = this.getDistanceTo(tile);
				if (decayMemory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile) {
					auctionTiles.add(tile);
				}
				else {
					possibleTileGoals.add(tile);
				}
			}
			while (!holesInZone.isEmpty()) {
				TWEntity hole = holesInZone.poll();
				double distToHole = this.getDistanceTo(hole);
				if (decayMemory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole) {
					auctionHoles.add(hole);
				}
				else {
					possibleHoleGoals.add(hole);
				}
			}

			closestTile = new TWEntity[possibleTileGoals.size()];
			closestHole = new TWEntity[possibleHoleGoals.size()];
			closestTile = possibleTileGoals.toArray(closestTile);
			closestHole = possibleHoleGoals.toArray(closestHole);

			// Announce goals within set limit, goalAnnounceCount, then auction surplus targets
			int tileCount = this.carriedTiles.size();
			for (int i = 0; i < closestTile.length; i++) {
				if (tileCount >= 3 || i > goalAnnounceCount)
					auctionTiles.add(closestTile[i]);
				else {
					tileCount++;
					goals.add(closestTile[i]);
				}
			}
			for (int i = 0; i < closestHole.length; i++) {
				if (tileCount <= 0 || i > goalAnnounceCount)
					auctionHoles.add(closestHole[i]);
				else {
					tileCount--;
					goals.add(closestHole[i]);
				}
			}
			if (goals.size() >= 0) {
				TWEntity[] goalsArr = new TWEntity[goals.size()];
				goalsArr = goals.toArray(goalsArr);
				Message goalMessage = new TypedMessage(name, "ALL", "GOALS", goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}

			if (auctionTiles.size() >= 0) {
				TWEntity[] auctionArr = new TWEntity[auctionTiles.size()];
				auctionArr = auctionTiles.toArray(auctionArr);
				Message auctionTileMessage = new TypedMessage(name, "ALL", "AUCTION_TILE", new Object[] {auctionArr, agentZones[agentIdx]});
				this.getEnvironment().receiveMessage(auctionTileMessage);
			}
			if (auctionHoles.size() >= 0) {
				TWEntity[] auctionArr = new TWEntity[auctionHoles.size()];
				auctionArr = auctionHoles.toArray(auctionArr);
				Message auctionHoleMessage = new TypedMessage(name, "ALL", "AUCTION_HOLE", new Object[] {auctionArr, agentZones[agentIdx]});
				this.getEnvironment().receiveMessage(auctionHoleMessage);
			}
		}
	}

	public void assignZone(int startX, int startY, int width, int height) {
		int agentCount = 0;
		ArrayList<Message> messages = this.getEnvironment().getMessages();
		for (int i = 0; i < messages.size(); i++) {
			TypedMessage message = (TypedMessage) messages.get(i);
			if (message.getTo().equals("ALL") && message.getMessage().equals("MAP")) {
				agentCount++;
			}
		}

		agentZones = new Integer[agentCount];
		Int2D[] agentPos = new Int2D[agentCount];
		for (int i = 0; i < messages.size(); i++) {
			TypedMessage message = (TypedMessage) messages.get(i);
			if (message.getTo().equals("ALL") && message.getMessage().equals("MAP")) {
				agentPos[Character.getNumericValue(message.getFrom().charAt(message.getFrom().length() - 1)) - 1] = (Int2D) message.getObject()[1];
			}
		}
		// If environment height longer than width, divide into zones along height by
		// rows
		boolean[] agentAssigned = new boolean[agentCount];
		zoneByRows = width <= height;
		Int2D zoneDim;
		if (zoneByRows) {
			zoneDim = new Int2D(width, height / 3);

			// Assign closest agent for each zone
			for (int i = 0; i < agentCount; i++) {
				int[] distToZone = new int[agentCount];
				for (int j = 0; j < distToZone.length; j++) {
					if (!agentAssigned[j]) {
						distToZone[j] = Math.abs(agentPos[j].x - startX) + Math.abs(agentPos[j].y - ((zoneDim.y * i) + startY));
					}
					else {
						distToZone[j] = Parameters.xDimension + Parameters.yDimension;
					}
				}
				int closestAgent = 0;
				int closestDist = distToZone[closestAgent];
				for (int j = 1; j < distToZone.length; j++) {
					if (distToZone[j] < closestDist) {
						closestAgent = j;
						closestDist = distToZone[j];
					}
				}
				agentZones[closestAgent] = i;
				agentAssigned[closestAgent] = true;
			}
		}
		// Else divide into zones along width by columns
		else {
			zoneDim = new Int2D(width / 3, height);

			// Assign closest agent for each zone
			for (int i = 0; i < agentCount; i++) {
				int[] distToZone = new int[agentCount];
				for (int j = 0; j < distToZone.length; j++) {
					if (!agentAssigned[j]) {
						distToZone[j] = Math.abs(agentPos[j].x - ((zoneDim.x * i) + startX)) + Math.abs(agentPos[j].y - startY);
					}
					else {
						distToZone[j] = Parameters.xDimension + Parameters.yDimension;
					}
				}
				int closestAgent = 0;
				int closestDist = distToZone[closestAgent];
				for (int j = 1; j < distToZone.length; j++) {
					if (distToZone[j] < closestDist) {
						closestAgent = j;
						closestDist = distToZone[j];
					}
				}
				agentZones[closestAgent] = i;
				agentAssigned[closestAgent] = true;
			}
		}

		// Calculate bounds
		if (zoneByRows) {
			bounds[0] = new Int2D(startX, startY + (zoneDim.y * agentZones[agentIdx]));
			bounds[1] = new Int2D(startX + width, startY + (zoneDim.y * agentZones[agentIdx]));

			// Last zone covers entire remaining area
			if (agentZones[agentIdx] == agentZones.length - 1) {
				bounds[2] = new Int2D(startX + width, startY + height);
				bounds[3] = new Int2D(0, startY + height);
			}
			else {
				bounds[2] = new Int2D(startX + width, startY + (zoneDim.y * (agentZones[agentIdx] + 1)));
				bounds[3] = new Int2D(0, startY + (zoneDim.y * (agentZones[agentIdx] + 1)));
			}
		}
		else {
			bounds[0] = new Int2D(startX + (zoneDim.x * agentZones[agentIdx]), startY);

			// Last zone covers entire remaining area
			if (agentZones[agentIdx] == agentZones.length - 1) {
				bounds[1] = new Int2D(startX + width, startY);
				bounds[2] = new Int2D(startX + width, startY + height);
			}
			else {
				bounds[1] = new Int2D(startX + (zoneDim.x * (agentZones[agentIdx] + 1)), startY);
				bounds[2] = new Int2D(startX + (zoneDim.x * (agentZones[agentIdx] + 1)), startY + height);
			}

			bounds[3] = new Int2D(startX + (zoneDim.x * agentZones[agentIdx]), startY + height);
		}

		// Calculate anchors
		zoneDim = new Int2D(bounds[1].x - bounds[0].x, bounds[3].y - bounds[0].y);
		int horizontalAnchors = (int) Math.ceil(zoneDim.x / (Parameters.defaultSensorRange * 2.0 + 1));
		int verticalAnchors = (int) Math.ceil(zoneDim.y / (Parameters.defaultSensorRange * 2.0 + 1));
		anchors = new Int2D[horizontalAnchors * verticalAnchors];
		for (int i = 0, j = 0; i < verticalAnchors; i++) {
			Int2D[] tmpAnchors = new Int2D[horizontalAnchors];
			for (int k = 0; k < horizontalAnchors; k++) {
				int anchorX, anchorY;
				if (i == verticalAnchors - 1) {
					anchorY = bounds[2].y - Parameters.defaultSensorRange;
				}
				else {
					anchorY = bounds[0].y + Parameters.defaultSensorRange + (Parameters.defaultSensorRange * 2 + 1) * i;
				}
				if (k == horizontalAnchors - 1) {
					anchorX = bounds[2].x - Parameters.defaultSensorRange;
				}
				else {
					anchorX = bounds[0].x + Parameters.defaultSensorRange + (Parameters.defaultSensorRange * 2 + 1) * k;
				}
				tmpAnchors[k] = new Int2D(anchorX, anchorY);
			}

			// Reverse odd rowed anchors for more efficient exploration
			if (i % 2 != 0) {
				for (int k = horizontalAnchors - 1; k >= 0; k--) {
					anchors[j] = tmpAnchors[k];
					j++;
				}
			}
			else {
				for (int k = 0; k < horizontalAnchors; k++) {
					anchors[j] = tmpAnchors[k];
					j++;
				}
			}
		}

		return;
	}

	public void mergeContracts(PriorityQueue<TWEntity> queue, TWEntity[] contractObj, int contractZone) {
		if (this.agentZones[agentIdx] != contractZone &&
			Math.abs(this.agentZones[agentIdx] - contractZone) <= maxAssistZoneDistance) {
			for (int i = 0; i < contractObj.length; i++) {
				double distToObj = this.getDistanceTo(contractObj[i]);
				if (!(decayMemory.getEstimatedRemainingLifetime(contractObj[i], this.objectLifetimeThreshold) <= distToObj)) {
					queue.add(contractObj[i]);
				}
			}
		}
	}

	public double getTSPDistance(TWEntity o) {
		double oDist = getDistanceTo(o);
		// Modifies Manhattan distance by lifetime remaining, so between two equidistant objects, the one with a shorter lifetime is closer
		if (this.TSPHeuristic) {
			oDist *= decayMemory.getEstimatedRemainingLifetime(o, 1.0)/Parameters.lifeTime;
		}
		return oDist;
	}

	@Override
	protected TWThought think() {
		// If bounds and anchors are not decided,
		// divide up map into as many zones as agents and
		// and allocate zones offline based on nearest agent positions.
		// Further divide each zone into smaller areas with centered
		// anchors based on sensing range.
		if (bounds[0] == null) {
			assignZone(0, 0, Parameters.xDimension, Parameters.yDimension);
			tilesInZone = decayMemory.getNearbyObjectsWithinBounds(bounds, new TWTile().getClass());
			holesInZone = decayMemory.getNearbyObjectsWithinBounds(bounds, new TWHole().getClass());
			while (!tilesInZone.isEmpty()) {
				TWEntity tile = tilesInZone.poll();
				double distToTile = this.getDistanceTo(tile);
				if (!(decayMemory.getEstimatedRemainingLifetime(tile, this.objectLifetimeThreshold) <= distToTile)) {
					possibleTileGoals.add(tile);
				}
			}
			while (!holesInZone.isEmpty()) {
				TWEntity hole = holesInZone.poll();
				double distToHole = this.getDistanceTo(hole);
				if (!(decayMemory.getEstimatedRemainingLifetime(hole, this.objectLifetimeThreshold) <= distToHole)) {
					possibleHoleGoals.add(hole);
				}
			}
			closestTile = new TWEntity[possibleTileGoals.size()];
			closestHole = new TWEntity[possibleHoleGoals.size()];
			closestTile = possibleTileGoals.toArray(closestTile);
			closestHole = possibleHoleGoals.toArray(closestHole);
		}

		// Merge all shared maps before any further deliberation
		ArrayList<Message> messages = this.getEnvironment().getMessages();
		for (int i = 0; i < messages.size(); i++) {
			TypedMessage message = (TypedMessage) messages.get(i);
			if (!message.getFrom().equals(this.name) &&
				message.getTo().equals("ALL") &&
				message.getMessage().equals("MAP")) {
				decayMemory.mergeMemory((TWAgentPercept[][]) message.getObject()[0], (Int2D) message.getObject()[1]);
			}
		}

		// Check environment for available contracts
		Comparator<TWEntity> distHeur = new Comparator<TWEntity>() {
			public int compare(TWEntity o1, TWEntity o2) {
				   return (int) (getTSPDistance(o1) - getTSPDistance(o2));
			}
		};
		PriorityQueue<TWEntity> assistableTiles = new PriorityQueue<TWEntity>(10, distHeur);
		PriorityQueue<TWEntity> assistableHoles = new PriorityQueue<TWEntity>(10, distHeur);
		for (int i = 0; i < messages.size(); i++) {
			TypedMessage message = (TypedMessage) messages.get(i);
			if (!message.getFrom().equals(this.name) &&
				message.getTo().equals("ALL")) {
				if (message.getMessage().equals("AUCTION_TILE")) {
					mergeContracts(assistableTiles, (TWEntity[]) message.getObject()[0], (int) message.getObject()[1]);
				}
				else if (message.getMessage().equals("AUCTION_HOLE")) {
					mergeContracts(assistableHoles, (TWEntity[]) message.getObject()[0], (int) message.getObject()[1]);
				}
			}
		}

		// Remove announced goals from goal and contract lists to prevent goal collisions
		for (int i = 0; i < messages.size(); i++) {
			TypedMessage message = (TypedMessage) messages.get(i);
			if (!message.getFrom().equals(this.name) &&
				message.getTo().equals("ALL") &&
				message.getMessage().equals("GOALS")) {
				TWEntity[] announcedGoals = (TWEntity[]) message.getObject();
				for (int j = 0; j < announcedGoals.length; j++) {
					if (announcedGoals[j] instanceof TWTile) {
						assistableTiles.remove(announcedGoals[j]);
						possibleTileGoals.remove(announcedGoals[j]);
					}
					else if (announcedGoals[j] instanceof TWHole) {
						assistableHoles.remove(announcedGoals[j]);
						possibleHoleGoals.remove(announcedGoals[j]);
					}
				}
			}
		}

		// Default Mode
		mode = Mode.EXPLORE;

		// Refueling takes utmost priority if fuel station already found and low on fuel
		if (decayMemory.getFuelStation() != null && this.getDistanceTo(decayMemory.getFuelStation().x, decayMemory.getFuelStation().y) >= this.fuelLevel * fuelTolerance) {
			mode = Mode.REFUEL;
		}
		// If fuel station not yet found, exploration takes highest priority
		else if (decayMemory.getFuelStation() == null) {
			mode = Mode.EXPLORE;
		}
		else if (this.fuelLevel <= this.hardFuelLimit) {
			// Extremely rare scenario where fuel station is not found during first exploration phase
			// Agent waits until other agents finishes exploring their zones and hopefully finds the fuel station
			// In the event fuel station is in this agent's zone, other agents have to assist exploring
			// the remaining parts of this zone, hopefully with enough fuel to spare
			// (ASSIST_EXPLORE not programmed in yet, requires broadcasting remaining exploration map to environment)
			if (decayMemory.getFuelStation() == null) {
				mode = Mode.WAIT;
			}
			else {
				mode = Mode.REFUEL;
			}
		}
		// If no tile and tile nearby, collect tile else explore
		else if (!this.hasTile()) {
			if (closestTile.length > 0) {
				mode = Mode.COLLECT;
			}
			else if (allowAssistance && !assistableTiles.isEmpty()) {
				mode = Mode.ASSIST_COLLECT;
			}
		}
		// If agent has tile and there is a hole nearby, prioritize filling hole
		else if (closestHole.length > 0) {
			if (closestTile.length == 0 ||
				(getTSPDistance(closestHole[0]) <= getTSPDistance(closestTile[0])) ||
				this.carriedTiles.size() >= 3) {
				mode = Mode.FILL;
			}
			else {
				mode = Mode.COLLECT;
			}
		}
		// If not at maximum number of tiles and there is only tile(s) nearby, collect tile(s)
		else if (closestTile.length > 0 && this.carriedTiles.size() < 3) {
			mode = Mode.COLLECT;
		}
		// If no tiles and holes in own zone, but there are assistable holes in a neighboring zone,
		// assist agent in neighboring zone to fill holes
		else if (allowAssistance && !assistableHoles.isEmpty()) {
			mode = Mode.ASSIST_FILL;
		}

		// Deletes plan and reevaluate goals every step rather than checking for new/decayed tiles/holes/obstacles
		// and changing existing plan
		planner.getGoals().clear();
		planner.voidPlan();

		// Always checks if possible to pickup/fill/refuel even when prioritizing
		// exploration
		Object curLocObj = this.memory.getMemoryGrid().get(x, y);
		if (curLocObj instanceof TWHole &&
			this.getEnvironment().canPutdownTile((TWHole) curLocObj, this)) {
			mode = Mode.REACT_FILL;

			// Announce goal as there is possibility target is not in purview of own's zone and is encountered enroute to or from refueling
			// If so, there may be a possibility of goal collision
			TWEntity[] goalsArr = new TWEntity[] {this.decayMemory.objects[x][y].getO()};
			Message goalMessage = new TypedMessage(name, "ALL", "GOALS", goalsArr);
			this.getEnvironment().receiveMessage(goalMessage);

			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.PUTDOWN, null);
		}
		else if (curLocObj instanceof TWTile &&
				 this.carriedTiles.size() < 3 &&
				 this.getEnvironment().canPickupTile((TWTile) curLocObj, this))
		{
			mode = Mode.REACT_COLLECT;

			TWEntity[] goalsArr = new TWEntity[] {this.decayMemory.objects[x][y].getO()};
			Message goalMessage = new TypedMessage(name, "ALL", "GOALS", goalsArr);
			this.getEnvironment().receiveMessage(goalMessage);

			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.PICKUP, null);
		}
		// If stumble upon fuel station, refuel if below 75% fuel.
		// This is different from the fuel management mechanism using the fuelTolerance threshold.
		else if (curLocObj instanceof TWFuelStation &&
				 this.fuelLevel < (0.75 * Parameters.defaultFuelLevel))
		{
			planner.getGoals().add(new Int2D(this.x, this.y));
			return new TWThought(TWAction.REFUEL, null);
		}
		// Modes which require a TWDirection and plan to be generated
		else {
			// getMemory().getClosestObjectInSensorRange(Tile.class);
			if (mode == Mode.EXPLORE) {
				// Collect exploration scores for all anchors
				Int2D anchorGoal = anchors[0];
				Double max_score = Double.NEGATIVE_INFINITY;

				for (int i = 0; i < anchors.length; i++) {
					Double curExplorationScore = decayMemory.getAnchorExplorationScore(anchors[i]);
					Double distToAnchor = this.getDistanceTo(anchors[i].x, anchors[i].y);

					if (curExplorationScore > max_score ||
					   ((curExplorationScore.equals(max_score)) &&
						(distToAnchor < this.getDistanceTo(anchorGoal.x, anchorGoal.y)))
					   ) {
						max_score = curExplorationScore;

						// If blocked, source for alternative positions with highest exploration score
						if (decayMemory.isCellBlocked(anchors[i].x, anchors[i].y)) {
							ArrayList<Int2D> alternativeAnchors = new ArrayList<Int2D>();
							ArrayList<Double> alternativeScores = new ArrayList<Double>();
							for (int j = -1; j <= 1; j++) {
								for (int k = -1; k <= 1; k++) {
									if (anchors[i].x + j < this.getEnvironment().getxDimension() &&
										anchors[i].y + k < this.getEnvironment().getyDimension() &&
										anchors[i].x - j >= 0 &&
										anchors[i].y + k >= 0 &&
										!decayMemory.isCellBlocked(anchors[i].x + j, anchors[i].y + k)) {
										alternativeAnchors.add(new Int2D(anchors[i].x + j, anchors[i].y + k));
										alternativeScores.add(decayMemory.getAnchorExplorationScore(alternativeAnchors.get(alternativeAnchors.size() - 1)));
									}
								}
							}
							anchorGoal = alternativeAnchors.get(0);
							int max_alt = 0;
							for (int j = 0; j < alternativeAnchors.size(); j++) {
								if (alternativeScores.get(j) > alternativeScores.get(max_alt)) {
									anchorGoal = alternativeAnchors.get(j);
								}
							}
						}
						else {
							anchorGoal = anchors[i];
						}
					}
				}

				planner.getGoals().add(anchorGoal);
			}
			else if (mode == Mode.REFUEL) {
				planner.getGoals().add(decayMemory.fuelStation);
			}
			else if (mode == Mode.COLLECT) {
				planner.getGoals().add(new Int2D(closestTile[0].getX(), closestTile[0].getY()));
			}
			else if (mode == Mode.FILL) {
				planner.getGoals().add(new Int2D(closestHole[0].getX(), closestHole[0].getY()));
			}
			else if (mode == Mode.ASSIST_COLLECT) {
				planner.getGoals().add(new Int2D(assistableTiles.peek().getX(), assistableTiles.peek().getY()));

				// Broadcast goal being assisted to indicate contract is no longer available
				TWEntity[] goalsArr = new TWEntity[] {assistableTiles.peek()};
				Message goalMessage = new TypedMessage(name, "ALL", "GOALS", goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}
			else if (mode == Mode.ASSIST_FILL) {
				planner.getGoals().add(new Int2D(assistableHoles.peek().getX(), assistableHoles.peek().getY()));

				// Broadcast goal being assisted to indicate contract is no longer available
				TWEntity[] goalsArr = new TWEntity[] {assistableHoles.peek()};
				Message goalMessage = new TypedMessage(name, "ALL", "GOALS", goalsArr);
				this.getEnvironment().receiveMessage(goalMessage);
			}
			else if (mode == Mode.WAIT) {
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			}

			planner.generatePlan();
			if (!planner.hasPlan()) {
				return new TWThought(TWAction.MOVE, TWDirection.Z);
			}
			return new TWThought(TWAction.MOVE, planner.execute());
		}
	}

	@Override
	protected void act(TWThought thought) {
		// You can do:
		// move(thought.getDirection())
		// pickUpTile(Tile)
		// putTileInHole(Hole)
		// refuel()
		Int2D curGoal = planner.getCurrentGoal();

		try {
			switch (thought.getAction()) {
			case MOVE:
				move(thought.getDirection());
				break;
			case PICKUP:
				pickUpTile((TWTile) memory.getMemoryGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
			case PUTDOWN:
				putTileInHole((TWHole) memory.getMemoryGrid().get(this.x, this.y));
				planner.getGoals().clear();
				break;
			case REFUEL:
				refuel();
				planner.getGoals().clear();
				break;
			}
		}
		catch (CellBlockedException ex) {
			// Cell is blocked, replan?
			System.out.println("Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
		}
		System.out.println("Step " + this.getEnvironment().schedule.getSteps());
		System.out.println(name + " score: " + this.score);
		System.out.println("Assigned Zone: " + Integer.toString(agentZones[agentIdx]));
		System.out.println("Mode: " + mode.name());
		System.out.println("Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
		if (curGoal != null) {
			System.out.println("Goal: " + curGoal.x + ", " + curGoal.y);
		}
		else
			System.out.println("Goal: WAIT");
		System.out.println("Tiles: " + this.carriedTiles.size());
		System.out.println("Fuel Level: " + this.fuelLevel);
		System.out.println("");
	}

	@Override
	public String getName() {
		return name;
	}
}