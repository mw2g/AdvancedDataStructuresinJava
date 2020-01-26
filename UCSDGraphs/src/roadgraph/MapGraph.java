/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private Map<GeographicPoint,List<Edge>> map;
	
	/** WEEK 3
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		map = new HashMap<GeographicPoint,List<Edge>>();
	}
	
	/** WEEK 3
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return map.size();
	}
	
	/** WEEK 3
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return map.keySet();
	}
	
	/** WEEK 3
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges(){
		int numEdges = 0;
		
		for (GeographicPoint geographicPoint : map.keySet())
			numEdges += map.get(geographicPoint).size();

		return numEdges;
	}

	
	
	/** WEEK 3 
	 * Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		List<Edge> edgeList = new ArrayList<>();
		
		if(location !=null && map.putIfAbsent(location, edgeList) == null)
			return true;
		
		return false;
	}
	
	/** WEEK 3
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
						String roadType, double length) throws IllegalArgumentException {

		Edge edge = new Edge(to, roadName, roadType, length);
		map.get(from).add(edge);
	}
	

	/** WEEK 3
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** WEEK 3
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		if (start == null || goal == null || nodeSearched == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		toExplore.add(start);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			GeographicPoint curr = toExplore.remove();
			if (curr.getX() == goal.getX() && curr.getY() == goal.getY()) {
				found = true;
				break;
			}
			List<GeographicPoint> neighbors = getNeighbors(curr);
			ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				GeographicPoint next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					nodeSearched.accept(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		return reconstructPath(start, goal, parentMap);
	}
	
	/** WEEK 3
	 * Reconstruct the path for return in bfs method
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap The path, on which did search from start to goal
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	private LinkedList<GeographicPoint> reconstructPath(GeographicPoint start, GeographicPoint goal, 
														HashMap<GeographicPoint, GeographicPoint> parentMap){
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	
	/** WEEK 3
	 * Finding neighbors for given point
	 * 
	 * @param point Point to find neighbors 
	 * @return The list of neighbors for given point.
	 */
	private List<GeographicPoint> getNeighbors(GeographicPoint point) {
		
		if(point == null)
			return null;
		
		List<GeographicPoint> neighbors = new ArrayList<>();
		for (Edge edge : map.get(point)) {
			neighbors.add(edge.end);
		}
		return neighbors;
	}
	
	/** WEEK 3
	 * Finding edge between two points
	 * 
	 * @param point1 Point 1 to find neighbors 
	 * @param point2 Point 2 to find neighbors 
	 * @return Edge between two points
	 */
	private Edge getEdge(GeographicPoint point1, GeographicPoint point2) {
		
		if(point1 == null || point2 == null)
			return null;
		
		for (Edge edge : map.get(point1)) {
			if(edge.getEnd().equals(point2))
				return edge;
		}
		return null;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start == null || goal == null || nodeSearched == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		PriorityQueue<List<Object>> toExplore = new PriorityQueue<>(new Comparator<List<Object>>() {

			@Override
			public int compare(List<Object> x, List<Object> y) {
				
				int result = Integer.compare((int)x.get(2), (int)y.get(2));
				
				if(result == 0)
					result = Double.compare((double)x.get(1), (double)y.get(1));
				
				return result;
			}
		});
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		List<Object> pD = Arrays.asList(start, 0.00);
		toExplore.add(pD);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			List<Object> curr = toExplore.remove();
			if (!visited.contains((GeographicPoint) curr.get(0))) {
				visited.add((GeographicPoint) curr.get(0));
				if (goal.equals(curr.get(0))) {
					found = true;
					break;
				}
				List<GeographicPoint> neighbors = getNeighbors((GeographicPoint) curr.get(0));
				ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
				while (it.hasPrevious()) {
					GeographicPoint next = it.previous();
					if (!visited.contains(next)) {
						nodeSearched.accept(next);
						double allDist = ((double) curr.get(1)) + getEdge((GeographicPoint) curr.get(0), next).getLength();
						double distanceNext = 99999999.99;
						for (List<Object> exp : toExplore) {
							if(next.equals(exp.get(0))) 
								distanceNext = (double) exp.get(1);
						}
						if (allDist  < distanceNext) {
							parentMap.put(next, (GeographicPoint) curr.get(0));
							int roadClass = getEdge((GeographicPoint) curr.get(0), next).getRoadClass();
							List<Object> pD1 = Arrays.asList(next, allDist, roadClass);
							toExplore.add(pD1);
						}
					}
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}
		
		System.out.println("Dijkstra visited (" + visited.size() + ") " + visited);
		return reconstructPath(start, goal, parentMap);
		
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null || nodeSearched == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		PriorityQueue<List<Object>> toExplore = new PriorityQueue<>(new Comparator<List<Object>>() {
			
			@Override
			public int compare(List<Object> x, List<Object> y) {
				
				int result = Integer.compare((int)x.get(3), (int)y.get(3));
				
				if(result == 0)
					result = Double.compare((double)x.get(2), (double)y.get(2));
				
				if(result == 0)
					result = Double.compare((double)x.get(1), (double)y.get(1));
				
				return result;
			}
		});
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		double goalDist = start.distance(goal);
		List<Object> pD = Arrays.asList(start, 0.00, goalDist);
		toExplore.add(pD);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			List<Object> curr = toExplore.remove();
			if (!visited.contains((GeographicPoint) curr.get(0))) {
				visited.add((GeographicPoint) curr.get(0));
				if (goal.equals(curr.get(0))) {
					found = true;
					break;
				}
				List<GeographicPoint> neighbors = getNeighbors((GeographicPoint) curr.get(0));
				ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
				while (it.hasPrevious()) {
					GeographicPoint next = it.previous();
					if (!visited.contains(next)) {
						nodeSearched.accept(next);
						int roadClass = getEdge((GeographicPoint) curr.get(0), next).getRoadClass();
						double allDist = ((double) curr.get(1)) + getEdge((GeographicPoint) curr.get(0), next).getLength();
						double distanceNext = 99999999.99;
						for (List<Object> exp : toExplore) {
							if(next.equals(exp.get(0))) 
								distanceNext = (double) exp.get(1);
						}
						if (allDist  < distanceNext) {
							goalDist = goal.distance(next);
							parentMap.put(next, (GeographicPoint) curr.get(0));
							List<Object> pD1 = Arrays.asList(next, allDist,  goalDist, roadClass);
							toExplore.add(pD1);
						}
					}
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}
		
		System.out.println("A* visited (" + visited.size() + ") " + visited);
		return reconstructPath(start, goal, parentMap);
		
	}
	
	
	/** WEEK 3
	 * A local class which represents road between intersections (edges between nodes)
	 */
	class Edge{
		private GeographicPoint end;
		private String roadName;
		private String roadType;
		private double length;
		private int roadClass;
		
		/** WEEK 3
		 * Create a new edge
		 * @param roadName The name of road
		 * @param roadType The type of road 
		 * @param length The length of road 
		 */
		public Edge(GeographicPoint to, String roadName, String roadType, double length) {
			this.end = to;
			this.setRoadName(roadName);
			this.setRoadType(roadType);
			this.length = length;
			switch (roadType) {
			case "motorway":
				setRoadClass(1);
				break;
			case "motorway_link":
				setRoadClass(2);
				break;
			case "primary":
				setRoadClass(3);
				break;
			case "secondary":
				setRoadClass(4);
				break;
			case "tertiary":
				setRoadClass(5);
				break;
			case "city street":
				setRoadClass(6);
				break;
			case "connector":
				setRoadClass(7);
				break;
			case "residential":
				setRoadClass(8);
				break;
			case "living_street":
				setRoadClass(9);
				break;

			default:
				setRoadClass(10);
				break;
			}
		}

		public double getLength() {
			return length;
		}
		
		public GeographicPoint getEnd() {
			return end;
		}

		public String getRoadType() {
			return roadType;
		}

		public void setRoadType(String roadType) {
			this.roadType = roadType;
		}

		public String getRoadName() {
			return roadName;
		}

		public void setRoadName(String roadName) {
			this.roadName = roadName;
		}

		public int getRoadClass() {
			return roadClass;
		}

		public void setRoadClass(int roadClass) {
			this.roadClass = roadClass;
		}

	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
//		List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		
		
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
//		System.out.println(testroute);
//		System.out.println(testroute.size());
//		System.out.println(testroute2);
//		System.out.println(testroute2.size());
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
//		System.out.println(testroute);
//		System.out.println(testroute.size());
//		System.out.println(testroute2);
//		System.out.println(testroute2.size());
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
//		System.out.println(testroute);
//		System.out.println(testroute.size());
//		System.out.println(testroute2);
//		System.out.println(testroute2.size());
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		System.out.println(route);
		System.out.println(route.size());
		System.out.println(route2);
		System.out.println(route2.size());
		
	}
	
}
