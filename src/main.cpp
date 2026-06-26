#include "Graph.h"
#include "Parser.h"
#include "Algorithms.h"
#include "Benchmark.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

void exportPath(Graph& g,
                const std::vector<long long>& path,
                double dist, const std::string& algo) {
    std::ofstream f("output/path_output.json");
    if (!f.is_open()) {
        std::cerr << "Error: Could not open output/path_output.json for writing." << std::endl;
        return;
    }
    f << "{\n";
    f << "  \"algo\": \"" << algo << "\",\n";
    f << "  \"dist_km\": " << dist << ",\n";
    f << "  \"path\": [\n";
    for (size_t i = 0; i < path.size(); i++) {
        auto& n = g.nodes[path[i]];
        f << "    [" << n.lat
          << ", " << n.lon << "]";
        if (i + 1 < path.size()) f << ",";
        f << "\n";
    }
    f << "  ]\n}\n";
}

int main() {
    std::cout << "Road Network Pathfinder" << std::endl;
    std::cout << "Loading OSM data from data/city.osm..." << std::endl;
    
    Graph g = parseOSM("data/Nagpur.osm");
    if (g.nodes.empty()) {
        std::cerr << "Error: No nodes loaded. Make sure data/Nagpur.osm is present and valid." << std::endl;
        return 1;
    }

    long long totalEdges = 0;
    for (auto& [u, neighbors] : g.adj) {
        totalEdges += neighbors.size();
    }
    totalEdges /= 2;

    std::cout << "Loaded: " << g.nodes.size() << " nodes | " << totalEdges << " edges" << std::endl << std::endl;

    double srcLat, srcLon, dstLat, dstLon;
    std::cout << "Enter source latitude:  ";
    if (!(std::cin >> srcLat)) return 1;
    std::cout << "Enter source longitude: ";
    if (!(std::cin >> srcLon)) return 1;
    std::cout << "Enter dest latitude:    ";
    if (!(std::cin >> dstLat)) return 1;
    std::cout << "Enter dest longitude:   ";
    if (!(std::cin >> dstLon)) return 1;

    std::cout << "\nSnapping to nearest nodes..." << std::endl;
    long long srcNode = nearestNode(g, srcLat, srcLon);
    long long dstNode = nearestNode(g, dstLat, dstLon);

    if (srcNode == -1 || dstNode == -1) {
        std::cerr << "Error: Could not find nearest nodes on the road network." << std::endl;
        return 1;
    }

    double srcSnapDist = haversine(srcLat, srcLon, g.nodes[srcNode].lat, g.nodes[srcNode].lon);
    double dstSnapDist = haversine(dstLat, dstLon, g.nodes[dstNode].lat, g.nodes[dstNode].lon);

    std::cout << "Source: node " << srcNode << " (" << srcSnapDist << "km away)" << std::endl;
    std::cout << "Dest:   node " << dstNode << " (" << dstSnapDist << "km away)" << std::endl;

    std::cout << "\nRunning algorithms..." << std::endl;

    // Run Dijkstra
    BenchResult dRes = benchDijkstra(g, srcNode, dstNode);
    // Run A*
    BenchResult aRes = benchAStar(g, srcNode, dstNode);
    
    // Run BFS with node counting
    long long bfsVisited = 0;
    auto startBfs = std::chrono::high_resolution_clock::now();
    // Helper that counts nodes visited for BFS
    std::unordered_map<long long, long long> prev;
    std::unordered_set<long long> visited;
    std::queue<long long> q;
    q.push(srcNode);
    visited.insert(srcNode);
    bool bfsFound = false;
    while (!q.empty()) {
        long long u = q.front(); q.pop();
        bfsVisited++;
        if (u == dstNode) {
            bfsFound = true;
            break;
        }
        auto it = g.adj.find(u);
        if (it != g.adj.end()) {
            for (auto [v, w] : it->second) {
                if (!visited.count(v)) {
                    visited.insert(v);
                    prev[v] = u;
                    q.push(v);
                }
            }
        }
    }
    std::vector<long long> bfsPath;
    if (bfsFound || srcNode == dstNode) {
        for (long long cur = dstNode; cur != srcNode; cur = prev[cur]) {
            bfsPath.push_back(cur);
        }
        bfsPath.push_back(srcNode);
        std::reverse(bfsPath.begin(), bfsPath.end());
    }
    auto endBfs = std::chrono::high_resolution_clock::now();
    long long bfsUs = std::chrono::duration_cast<std::chrono::microseconds>(endBfs - startBfs).count();

    // Compute BFS distance
    double bfsDist = 0.0;
    for (size_t i = 0; i + 1 < bfsPath.size(); ++i) {
        bfsDist += haversine(g.nodes[bfsPath[i]].lat, g.nodes[bfsPath[i]].lon,
                             g.nodes[bfsPath[i+1]].lat, g.nodes[bfsPath[i+1]].lon);
    }

    std::cout << "===========================================================\n\n";
    std::cout << "                     BENCHMARK RESULTS                     \n";

    // Helper lambda to print blocks cleanly
    auto printBlock = [](const std::string& name, double dist, long long nodes, double ms, bool found) {
        std::cout << "[ " << name << " ]\n";
        if (!found || dist >= 1e17) {
            std::cout << "  - Status:        No Path Found\n\n";
        } else {
            std::cout << "  - Distance:      " << std::fixed << std::setprecision(2) << dist << " km\n";
            std::cout << "  - Nodes Visited: " << nodes << "\n";
            std::cout << "  - CPU Time:      " << std::fixed << std::setprecision(2) << ms << " ms\n\n";
        }
    };

    printBlock("Dijkstra", dRes.distKm, dRes.nodesVisited, dRes.timeUs / 1000.0, dRes.distKm < 1e17);
    printBlock("A* (A-Star)", aRes.distKm, aRes.nodesVisited, aRes.timeUs / 1000.0, aRes.distKm < 1e17);
    printBlock("BFS", bfsDist, bfsVisited, bfsUs / 1000.0, !bfsPath.empty() || srcNode == dstNode);

    if (dRes.distKm < 1e17 && aRes.distKm < 1e17 && dRes.nodesVisited > 0) {
        double reduction = (1.0 - (double)aRes.nodesVisited / dRes.nodesVisited) * 100.0;
        std::cout << "-> A* Algorithm visited " << std::fixed << std::setprecision(0) << reduction 
                  << "% fewer nodes than Dijkstra!\n\n";
    }
    std::cout << "===========================================================\n\n";

    // Export the A* path
    if (aRes.distKm < 1e17) {
        exportPath(g, aRes.path, aRes.distKm, "A*");
        std::cout << "Path exported to output/path_output.json" << std::endl;
        std::cout << "Open map.html in your browser to visualize." << std::endl;
    } else if (dRes.distKm < 1e17) {
        exportPath(g, dRes.path, dRes.distKm, "Dijkstra");
        std::cout << "Path exported to output/path_output.json" << std::endl;
        std::cout << "Open map.html in your browser to visualize." << std::endl;
    } else {
        std::cout << "\nNo valid path found to export." << std::endl;
    }

    return 0;
}