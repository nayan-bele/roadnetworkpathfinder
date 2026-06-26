# Road Network Pathfinder

This project implements and compares three fundamental graph-search algorithms applied to a real, large-scale problem: finding the shortest driving route between two locations on a road network.
Road data is sourced from OpenStreetMap (OSM), parsed from .osm files, and loaded into an in-memory weighted directed graph.

## Algorithms

### 1. Dijkstra's Algorithm

Dijkstra's algorithm computes the single-source shortest path in a graph with non-negative edge weights. It maintains a min-heap (priority queue) of nodes ordered by their current best-known distance from the source, expanding the cheapest frontier at each step.

**Time Complexity:** `O((V + E) log V)` with a binary heap  
**Space Complexity:** `O(V)`  
**Optimality:** Guaranteed to find the shortest path  

---

### 2. A\* (A-Star)

A\* extends Dijkstra by introducing an **admissible heuristic** `h(n)` — an estimate of the remaining cost from node `n` to the destination. The priority queue orders nodes by `f(n) = g(n) + h(n)`, where `g(n)` is the true cost from the source to `n`.

This project uses the **Haversine formula** as the heuristic, which computes the great-circle distance between two geographic coordinates. Because road distances can never be shorter than a straight line, Haversine never overestimates — making it admissible and the search result optimal.

**Time Complexity:** `O((V + E) log V)` worst-case (degenerates to Dijkstra); typically much better  
**Space Complexity:** `O(V)`  
**Optimality:** Guaranteed when the heuristic is admissible  

---

### 3. Breadth-First Search (BFS)

BFS explores the graph level by level (FIFO queue), finding the path with the **fewest edges** regardless of weight. On a road network where all roads have different lengths, BFS does not find the shortest distance — but it is included here for comparison and to demonstrate the difference between hop-count optimal and distance-optimal paths.

**Time Complexity:** `O(V + E)`  
**Space Complexity:** `O(V)`  
**Optimality:** Optimal for unweighted graphs only  

---
### 
The Dataset
* **Location:** Nagpur, India (City Extract)
* **Scale:** 171,037 intersections (nodes) and 44265 drivable road segments (edges).
* **Metrics:** All edge weights are calculated as true physical distances in kilometers using the **Haversine formula**, accounting for the curvature of the Earth.

---

## How it Works

The system accepts two GPS coordinates as input and snaps them to the nearest valid road intersections. It then executes the pathfinding algorithm, exports the resulting route, and visualizes it on a real-world map via map.html.

---
## Performance Matrix

To prove the efficiency of heuristic-based routing, the engine was benchmarked across three distinct geographic scenarios in Nagpur. 


| Query | Source ➔ Destination | Alg | Distance | Nodes Visited | Time | Use Case |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Q1: Dense Urban Center** | Sitabuldi ➔ Gandhibagh <br> `21.1440, 79.0810 -> 21.1450, 79.1150` | Dijkstra <br> A* <br> BFS | 3.87 km <br> 3.87 km <br> 4.40 km | 15,894 <br> 1,430 <br> 16,919 | 58.38ms <br> 36.30ms <br> 12.46ms | Shortest km <br> Shortest km (fast) <br> Fewest segments |
| **Q2: The Diagonal Marathon** | Medical Square ➔ Kalamna <br> `21.1120, 79.0920 -> 21.1750, 79.1350` | Dijkstra <br> A* <br> BFS | 10.01 km <br> 10.01 km <br> 12.90 km | 38,817 <br> 18,358 <br> 35,814 | 90.86ms <br> 69.14ms <br> 25.27ms | Shortest km <br> Shortest km (fast) <br> Fewest segments |
| **Q3: The Vertical Slice** | Sakkardara ➔ Jaripatka <br> `21.1150, 79.1050 -> 21.1800, 79.1000` | Dijkstra <br> A* <br> BFS | 7.82 km <br> 7.82 km <br> 10.84 km | 35,868 <br> 5,902 <br> 35,093 | 92.73ms <br> 48.62ms <br> 26.07ms | Shortest km <br> Shortest km (fast) <br> Fewest segments |

**Key Takeaway:** A* consistently found the exact same optimal route as Dijkstra while **slashing the search space by 80% to 90%**, effectively cutting CPU execution time.

---

## Architecture

```text
OSM XML → Parser → Adjacency List Graph
         → Dijkstra / A* / BFS
         → JSON export → Leaflet map
```
---

## What I learned

- **Dijkstra and A* are nearly identical in code:** The difference is just one line — the heuristic. But that one line cuts nodes visited by up to 91% because the algorithm now searches toward the destination instead of in all directions.

- **Haversine is real GPS distance — not Euclidean:** Using Euclidean on lat/lon gives wrong distances near the poles and doesn't account for Earth's curvature. Haversine uses spherical trigonometry to compute true distances.

- **Why BFS fails on physical maps:** BFS calculated a significantly longer path in every test (e.g., 12.90 km vs 10.01 km in Q2). Because it only minimizes the number of road segments, it will happily choose a massive highway over a shorter local route just to avoid an intersection.
---

## How to Run

### 1. Compile

```bash
g++ -O3 -std=c++17 src/main.cpp -o route_engine
```

### 2. Run

```bash
./route_engine
```

Enter the source and destination coordinates:

```text
Enter source latitude:  21.1150
Enter source longitude: 79.1050
Enter dest latitude:    21.1800
Enter dest longitude:   79.1000
```

### 3. Visualize the Route

Start a local HTTP server (required because browsers block local JSON files due to CORS):

```bash
python3 -m http.server
```

Then open:

```text
http://localhost:8000/map.html
```

---
