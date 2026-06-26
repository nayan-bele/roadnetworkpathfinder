# 🗺️ OpenStreetMap C++ Routing Engine

A high-performance, command-line routing engine built from scratch in pure C++17. This engine parses real-world city data directly from OpenStreetMap (OSM) XML files, constructs an optimized graph network, and calculates the mathematically perfect shortest route between GPS coordinates.

## 🚀 Features

* **Custom XML Parser:** Safely extracts node coordinates and drivable highway segments from raw `.osm` files, utilizing `try-catch` blocks to gracefully handle corrupted open-source data.
* **Optimized Graph Architecture:** Models city infrastructure using an Adjacency List (`std::unordered_map` + `std::vector`), ensuring $O(1)$ intersection lookups and allowing standard laptops to load entire cities without memory overflow.
* **Smart Coordinate Snapping:** Utilizes the **Haversine Formula** to snap arbitrary user GPS inputs (which might land in a park or a building) to the absolute nearest drivable physical street corner.
* **Algorithmic Benchmarking Suite:** Features an automated testing environment that compares the performance, CPU time, and search-space efficiency of Dijkstra's Algorithm, A* (A-Star), and Breadth-First Search (BFS).
* **GeoJSON Export:** Automatically translates the optimal calculated node path into standard GeoJSON format for instant visual rendering on web maps like Mapbox, Leaflet, or Google Maps.

---

## 📊 Performance Benchmarks (Nagpur Dataset)

The engine was tested on a densely populated real-world dataset (Nagpur, India). 
* **Nodes (Intersections) Indexed:** `171,037`
* **Edges (Road Segments) Mapped:** `36,705`

When calculating a 10.95 km route across the city, the **A* Algorithm** (utilizing a Haversine straight-line heuristic) demonstrated massive efficiency gains over standard Dijkstra:

| Algorithm | Distance | Nodes Visited | CPU Time |
| :--- | :--- | :--- | :--- |
| **Dijkstra** | 10.95 km | 33,069 | ~59 ms |
| **A\*** | 10.95 km | 3,768 | ~28 ms |
| **BFS** | 10.97 km * | N/A | ~16 ms |

*🏆 **Conclusion:** A\* found the exact same optimal route as Dijkstra while exploring **89% fewer intersections**, effectively cutting processing time in half. (*Note: BFS calculated a longer physical distance because it optimizes for fewest road segments, not actual kilometers).*

---

## 📂 Project Structure

```text
roadnetworkpathfinder/
├── src/
│   ├── main.cpp         # Entry point and CLI logic
│   ├── Graph.h          # Data structures (Nodes, Edges, Adjacency List)
│   ├── Parser.h         # OpenStreetMap XML parsing logic
│   ├── Algorithms.h     # Core routing math (Dijkstra, A*, BFS, Haversine)
│   ├── Benchmark.h      # Stopwatch and performance comparison wrappers
│   └── Export.h         # GeoJSON path formatting
├── data/
│   └── Nagpur.osm       # (Place your downloaded map data here)
└── output/
    └── path_output.json # The generated visual map file
