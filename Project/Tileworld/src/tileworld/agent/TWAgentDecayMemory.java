package tileworld.agent;

import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import javax.swing.text.html.HTMLDocument;
import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.environment.NeighbourSpiral;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * TWAgentMemory
 *
 * @author michaellees
 *
 *         Created: Apr 15, 2010 Copyright michaellees 2010
 *
 *         Description:
 *
 *         This class represents the memory of the TileWorld agents. It stores all objects which is has observed for a given period of time. You may want to develop an entirely new memory system by extending this one.
 *
 *         The memory is supposed to have a probabilistic decay, whereby an element is removed from memory with a probability proportional to the length of time the element has been in memory. The maximum length of time which the agent can remember is specified as MAX_TIME. Any memories beyond this are automatically removed.
 */
public class TWAgentDecayMemory extends TWAgentWorkingMemory {
	protected Schedule schedule;
	protected TWAgent me;

	protected final static int MAX_TIME = Parameters.lifeTime;

	protected ObjectGrid2D memoryGrid;
	protected Double[][] explorationScore; // Undiscovered = POSITIVE_INFINITY, OnSense = 0, OnDecay += 1
	protected TWAgentPercept[][] objects;
	protected Int2D fuelStation;

	protected int memorySize;

	protected TWAgentPercept[][] sensedMemory; // Keeps a record of the observable region to compare with incoming new records. Used for memory updates.
	protected HashMap<Class<?>, TWEntity> closestInSensorRange;
	static protected List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();
	protected List<TWAgent> neighbouringAgents = new ArrayList<TWAgent>();

	// x, y: the dimension of the grid
	public TWAgentDecayMemory(TWAgent moi, Schedule schedule, int x, int y) {
		super(moi, schedule, x, y);
		this.sensedMemory = new TWAgentPercept[Parameters.defaultSensorRange * 2 + 1][Parameters.defaultSensorRange * 2 + 1];
		this.me = moi;

		this.objects = new TWAgentPercept[x][y];
		this.explorationScore = new Double[x][y];
		for (int i = 0; i < x; i++) {
			for (int j = 0; j < y; j++) {
				explorationScore[i][j] = Double.POSITIVE_INFINITY;
			}
		}

		this.schedule = schedule;
		this.memoryGrid = new ObjectGrid2D(me.getEnvironment().getxDimension(), me.getEnvironment().getyDimension());
	}

	/**
	 * Called at each time step, updates the memory map of the agent. Note that some objects may disappear or be moved, in which case part of sensed may contain null objects
	 *
	 * Also note that currently the agent has no sense of moving objects, so an agent may remember the same object at two locations simultaneously.
	 *
	 * Other agents in the grid are sensed and passed to this function. But it is currently not used for anything. Do remember that an agent sense itself too.
	 *
	 * @param sensedObjects bag containing the sensed objects
	 * @param objectXCoords bag containing x coordinates of objects
	 * @param objectYCoords bag containing y coordinates of object
	 * @param sensedAgents  bag containing the sensed agents
	 * @param agentXCoords  bag containing x coordinates of agents
	 * @param agentYCoords  bag containing y coordinates of agents
	 */
	@Override
	public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
		// reset the closest objects for new iteration of the loop (this is short term observation memory if you like). It only lasts one timestep.
		closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);

		// must all be same size.
		assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());

		// me.getEnvironment().getMemoryGrid().clear(); // THis is equivalent to only having sensed area in memory
		this.decayMemory(); // You might want to think about when to call the decay function as well.

		// Backup observable region and clears the region in memory (i.e. refreshes observable region every update)
		int visibleX_min = me.getX() - Parameters.defaultSensorRange;
		int visibleY_min = me.getY() - Parameters.defaultSensorRange;
		for (int i = 0; i <= Parameters.defaultSensorRange * 2; i++) {
			for (int j = 0; j <= Parameters.defaultSensorRange * 2; j++) {
				int nx = visibleX_min + i;
				int ny = visibleY_min + j;
				// Horribly inefficient way of refreshing observable map because only way agent can sense surrounding is by the Bag of sensedObjects
				// instead of an array of the map. It's either looping through to delete old memory first, or looping through after updating to
				// compare which old object is not in list of sensedObjects. Needed because if a previously stored object within visible range
				// disappears, the original loop does not have any checks to remove it.
				if (sensedMemory[i][j] != null) {
					sensedMemory[i][j] = null;
					memorySize--;
				}
				if (nx >= 0 && ny >= 0 && nx < objects.length && ny < objects[1].length) {
					explorationScore[nx][ny] = 0.0;
					sensedMemory[i][j] = objects[nx][ny];
					objects[nx][ny] = null;
					memoryGrid.set(nx, ny, null);
				}
			}
		}

		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity o = (TWEntity) sensedObjects.get(i);
			if (!(o instanceof TWEntity)) {
				continue;
			}

			if (fuelStation == null && o instanceof TWFuelStation) {
				fuelStation = new Int2D(o.getX(), o.getY());
			}

			// If object previously already in memory, check if it's of same type.
			// If of same type set the time observed to that of the old one, if set the time observed to current time.
			int oX = o.getX() - visibleX_min;
			int oY = o.getY() - visibleY_min;
			TWAgentPercept prevObject = sensedMemory[oX][oY];
			if (sensedMemory[oX][oY] != null && prevObject.getO().getClass().equals(o.getClass())) {
				objects[o.getX()][o.getY()] = new TWAgentPercept(o, prevObject.getT());
				memoryGrid.set(o.getX(), o.getY(), o);
				memorySize++;
			}
			else {
				objects[o.getX()][o.getY()] = new TWAgentPercept(o, this.getSimulationTime());
				memoryGrid.set(o.getX(), o.getY(), o);
				memorySize++;
			}

			updateClosest(o);

		}
		// Agents are currently not added to working memory. Depending on how
		// communication is modelled you might want to do this.
		neighbouringAgents.clear();
		for (int i = 0; i < sensedAgents.size(); i++) {

			if (!(sensedAgents.get(i) instanceof TWAgent)) {
				assert false;
			}
			TWAgent a = (TWAgent) sensedAgents.get(i);
			if (a == null || a.equals(me)) {
				continue;
			}
			neighbouringAgents.add(a);
		}
	}

	public void mergeMemory(TWAgentPercept[][] objectsShared, Int2D agentPos) {
		int agentX = agentPos.x, agentY = agentPos.y;

		for (int i = 0; i < objects.length; i++) {
			for (int j = 0; j < objects[0].length; j++) {
				// if within sensing range of any agents, replace with latest percept
				if (i >= (agentX - Parameters.defaultSensorRange) && i <= (agentX + Parameters.defaultSensorRange) && j >= (agentY - Parameters.defaultSensorRange) && j <= (agentY + Parameters.defaultSensorRange)) {
					explorationScore[i][j] = 0.0;
					if (objectsShared[i][j] == null) {
						if (objects[i][j] != null) {
							objects[i][j] = null;
							memorySize--;
							memoryGrid.set(i, j, null);
						}
					}
					else {
						// If fuel station, store location
						if (fuelStation == null && objectsShared[i][j].getO() instanceof TWFuelStation) {
							fuelStation = new Int2D(objectsShared[i][j].getO().getX(), objectsShared[i][j].getO().getY());
						}

						if (objects[i][j] == null) {
							objects[i][j] = objectsShared[i][j];
							memorySize++;
							memoryGrid.set(i, j, objectsShared[i][j].getO());
						}
						else if (objectsShared[i][j].getT() > objects[i][j].getT()) {
							objects[i][j] = objectsShared[i][j];
							memoryGrid.set(i, j, objectsShared[i][j].getO());
						}
					}
				}
			}
		}
	}

	public TWAgent getNeighbour() {
		if (neighbouringAgents.isEmpty()) {
			return null;
		}
		else {
			return neighbouringAgents.get(0);
		}
	}

	@Override
	public void updateMemory(TWEntity[][] sensed, int xOffset, int yOffset) {
		for (int x = 0; x < sensed.length; x++) {
			for (int y = 0; y < sensed[x].length; y++) {
				objects[x + xOffset][y + yOffset] = new TWAgentPercept(sensed[x][y], this.getSimulationTime());
			}
		}
	}

	@Override
	public void decayMemory() {
		// put some decay on other memory pieces (this will require complete
		// iteration over memory though, so expensive.
		// This is a simple example of how to do this.
		for (int x = 0; x < this.objects.length; x++) {
			for (int y = 0; y < this.objects[x].length; y++) {
				TWAgentPercept currentMemory = objects[x][y];
				if (explorationScore[x][y] == 0.0)
					explorationScore[x][y] += 1.0;
				else
					explorationScore[x][y] *= 2.0;
				if (currentMemory != null && !(currentMemory.getO() instanceof TWFuelStation) && currentMemory.getT() < (schedule.getTime() - MAX_TIME)) {
					objects[x][y] = null;
					memoryGrid.set(x, y, null);
					memorySize--;
				}
			}
		}
	}

	@Override
	public void removeAgentPercept(int x, int y) {
		objects[x][y] = null;
	}

	@Override
	public void removeObject(TWEntity o) {
		removeAgentPercept(o.getX(), o.getY());
	}

	public double getSimulationTime() {
		return schedule.getTime();
	}

	@Override
	public TWTile getNearbyTile(int x, int y, double threshold) {
		return (TWTile) this.getNearbyObject(x, y, threshold, TWTile.class);
	}

	@Override
	public TWHole getNearbyHole(int x, int y, double threshold) {
		return (TWHole) this.getNearbyObject(x, y, threshold, TWHole.class);
	}

	@Override
	public int getMemorySize() {
		return memorySize;
	}

	protected TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {

		// If we cannot find an object which we have seen recently, then we want
		// the one with maxTimestamp
		double maxTimestamp = 0;
		TWObject o = null;
		double time = 0;
		TWObject ret = null;
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;

			if (me.getEnvironment().isInBounds(x, y) && objects[x][y] != null) {
				o = (TWObject) objects[x][y].getO();// get mem object
				if (type.isInstance(o)) {// if it's not the type we're looking for do nothing

					time = objects[x][y].getT();// get time of memory

					if (this.getSimulationTime() - time <= threshold) {
						// if we found one satisfying time, then return
						return o;
					}
					else if (time > maxTimestamp) {
						// otherwise record the timestamp and the item in case
						// it's the most recent one we see
						ret = o;
						maxTimestamp = time;
					}
				}
			}
		}

		// this will either be null or the object of Class type which we have
		// seen most recently but longer ago than now-threshold.
		return ret;
	}

	/**
	 * Returns the estimated remaining lifetime based on the time of memory
	 */
	public double getEstimatedRemainingLifetime(TWEntity o, double threshold) {
		if (objects[o.getX()][o.getY()] == null)
			return 0;
		else
			return (Parameters.lifeTime * threshold) - (this.getSimulationTime() - objects[o.getX()][o.getY()].getT());
	}

	protected PriorityQueue<TWEntity> getNearbyObjectsWithinBounds(Int2D[] bounds, Class<?> type) {
		TWObject o = null;
		PriorityQueue<TWEntity> ret = new PriorityQueue<TWEntity>(10,
			new Comparator<TWEntity>() {
				public int compare(TWEntity o1, TWEntity o2) {
					return (int) (((HybridPRSTWAgent)me).getTSPDistance(o1) - ((HybridPRSTWAgent)me).getTSPDistance(o2));
				}});
		int x, y;
		for (int i = bounds[0].x; i <= bounds[2].x; i++) {
			for (int j = bounds[0].y; j <= bounds[2].y; j++) {
				x = i;
				y = j;
	
				if (me.getEnvironment().isInBounds(x, y) &&
					objects[x][y] != null &&
					!(objects[x][y].getO() instanceof TWFuelStation)) {
					o = (TWObject) objects[x][y].getO();// get mem object
					if (type.isInstance(o)) {// if it's not the type we're looking for do nothing
						ret.add(o);
					}
				}
			}
		}
		return ret;
	}

	@Override
	public TWEntity getClosestObjectInSensorRange(Class<?> type) {
		return closestInSensorRange.get(type);
	}

	protected void updateClosest(TWEntity o) {
		assert (o != null);
		if (closestInSensorRange.get(o.getClass()) == null || me.closerTo(o, closestInSensorRange.get(o.getClass()))) {
			closestInSensorRange.put(o.getClass(), o);
		}
	}

	@Override
	public boolean isCellBlocked(int tx, int ty) {

		// no memory at all, so assume not blocked
		if (objects[tx][ty] == null) {
			return false;
		}

		TWEntity e = objects[tx][ty].getO();
		// is it an obstacle?
		return (e instanceof TWObstacle);
	}

	@Override
	public ObjectGrid2D getMemoryGrid() {
		return this.memoryGrid;
	}

	public TWAgentPercept[][] getAgentPercept() {
		return this.objects;
	}

	public Int2D getFuelStation() {
		return fuelStation;
	}

	public Double getAnchorExplorationScore(Int2D anchor) {
		Double score = 0.0;
		for (int i = (anchor.x - Parameters.defaultSensorRange); i <= (anchor.x + Parameters.defaultSensorRange); i++) {
			for (int j = (anchor.y - Parameters.defaultSensorRange); j <= (anchor.y + Parameters.defaultSensorRange); j++) {
				int x = i, y = j;
				// If i and j out of bounds, take the score of the mirrored cell
				// Prevents anchor points near boundary from being penalized because of having lesser valid cells
				if (i < 0) {
					x = -i;
				}
				else if (i >= Parameters.xDimension) {
					x = 2 * (Parameters.xDimension - 1) - i;
				}
				if (j < 0) {
					y = -j;
				}
				else if (j >= Parameters.yDimension) {
					y = 2 * (Parameters.yDimension - 1) - j;
				}
				score += explorationScore[x][y];
			}
		}
		return score;
	}
}
