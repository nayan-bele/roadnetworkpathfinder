#include "Graph.h"
#include "Parser.h"
#include "Algorithms.h"
#include "Benchmark.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

// ---------------------------------------------------------------------------
//  exportPath
//
//  Writes one algorithm's result to its own JSON file so the frontend can
//  load each algorithm independently (or all three at once for comparison).
//
//  Output file:  output/<algo>_output.json
//  Fields:
//    algo          – algorithm name string
//    dist_km       – total path length in kilometres
//    nodes_visited – number of nodes popped from the frontier
//    path          – array of [lat, lon] pairs (Leaflet order)
// ---------------------------------------------------------------------------
void exportPath(Graph& g,
                const std::vector<long long>& path,
                double distKm,
                long long nodesVisited,
                const std::string& algo)
{
    // Build a safe filename: "A*" → "astar", etc.
    std::string filename = "output/" + algo + "_output.json";

    std::ofstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Error: Could not open " << filename << " for writing.\n"
                  << "Make sure the 'output/' directory exists.\n";
        return;
    }

    f << std::fixed << std::setprecision(6);
    f << "{\n";
    f << "  \"algo\": \""          << algo          << "\",\n";
    f << "  \"dist_km\": "         << distKm        << ",\n";
    f << "  \"nodes_visited\": "   << nodesVisited  << ",\n";
    f << "  \"path\": [\n";

    for (size_t i = 0; i < path.size(); ++i) {
        auto& n = g.nodes[path[i]];
        f << "    [" << n.lat << ", " << n.lon << "]";
        if (i + 1 < path.size()) f << ",";
        f << "\n";
    }

    f << "  ]\n}\n";
    std::cout << "  -> Exported: " << filename << "\n";
}

int main()
{
    std::cout << "============================================================\n";
    std::cout << "              Road Network Pathfinder\n";
    std::cout << "============================================================\n\n";

    // ── Load OSM data ──────────────────────────────────────────────────
    std::cout << "Loading OSM data from data/Nagpur.osm...\n";
    Graph g = parseOSM("data/Nagpur.osm");

    if (g.nodes.empty()) {
        std::cerr << "Error: No nodes loaded. "
                  << "Make sure data/Nagpur.osm is present and valid.\n";
        return 1;
    }

    // Count undirected edges (each directed edge is stored twice)
    long long totalEdges = 0;
    for (auto& [u, neighbors] : g.adj)
        totalEdges += neighbors.size();
    totalEdges /= 2;

    std::cout << "Loaded: " << g.nodes.size() << " nodes | "
              << totalEdges << " edges\n\n";

    // ── Get source / destination from user ─────────────────────────────
    double srcLat, srcLon, dstLat, dstLon;
    std::cout << "Enter source latitude:  "; if (!(std::cin >> srcLat)) return 1;
    std::cout << "Enter source longitude: "; if (!(std::cin >> srcLon)) return 1;
    std::cout << "Enter dest latitude:    "; if (!(std::cin >> dstLat)) return 1;
    std::cout << "Enter dest longitude:   "; if (!(std::cin >> dstLon)) return 1;

    // ── Snap to nearest road nodes ─────────────────────────────────────
    std::cout << "\nSnapping to nearest road nodes...\n";
    long long srcNode = nearestNode(g, srcLat, srcLon);
    long long dstNode = nearestNode(g, dstLat, dstLon);

    if (srcNode == -1 || dstNode == -1) {
        std::cerr << "Error: Could not find nearby nodes on the road network.\n";
        return 1;
    }

    // haversine() returns metres — divide by 1000 for kilometres
    double srcSnapKm = haversine(srcLat, srcLon,
                                 g.nodes[srcNode].lat, g.nodes[srcNode].lon) / 1000.0;
    double dstSnapKm = haversine(dstLat, dstLon,
                                 g.nodes[dstNode].lat, g.nodes[dstNode].lon) / 1000.0;

    std::cout << "Source snapped: node " << srcNode
              << " (" << std::fixed << std::setprecision(3) << srcSnapKm << " km away)\n";
    std::cout << "Dest snapped:   node " << dstNode
              << " (" << std::fixed << std::setprecision(3) << dstSnapKm << " km away)\n\n";

    // ── Run Dijkstra ───────────────────────────────────────────────────
    std::cout << "Running Dijkstra...\n";
    BenchResult dRes = benchDijkstra(g, srcNode, dstNode);

    // ── Run A* ────────────────────────────────────────────────────────
    std::cout << "Running A*...\n";
    BenchResult aRes = benchAStar(g, srcNode, dstNode);

    // ── Run BFS (inline — BFS ignores weights, so we track manually) ───
    std::cout << "Running BFS...\n";

    long long bfsVisited = 0;
    std::vector<long long> bfsPath;
    double bfsDistKm = 0.0;

    {
        std::unordered_map<long long, long long> prev;
        std::unordered_set<long long> visited;
        std::queue<long long> q;

        q.push(srcNode);
        visited.insert(srcNode);

        bool found = false;
        auto bfsStart = std::chrono::high_resolution_clock::now();

        while (!q.empty()) {
            long long u = q.front(); q.pop();
            bfsVisited++;

            if (u == dstNode) { found = true; break; }

            auto it = g.adj.find(u);
            if (it == g.adj.end()) continue;
            for (auto [v, w] : it->second) {
                if (!visited.count(v)) {
                    visited.insert(v);
                    prev[v] = u;
                    q.push(v);
                }
            }
        }

        auto bfsEnd = std::chrono::high_resolution_clock::now();
        long long bfsUs = std::chrono::duration_cast<std::chrono::microseconds>(bfsEnd - bfsStart).count();

        if (found || srcNode == dstNode) {
            for (long long cur = dstNode; cur != srcNode; cur = prev[cur]) {
                if (prev.find(cur) == prev.end()) { bfsPath.clear(); break; }
                bfsPath.push_back(cur);
            }
            bfsPath.push_back(srcNode);
            std::reverse(bfsPath.begin(), bfsPath.end());

            // Compute geographic length of the BFS hop-path (metres → km)
            for (size_t i = 0; i + 1 < bfsPath.size(); ++i) {
                bfsDistKm += haversine(
                    g.nodes[bfsPath[i]].lat,   g.nodes[bfsPath[i]].lon,
                    g.nodes[bfsPath[i+1]].lat, g.nodes[bfsPath[i+1]].lon
                ) / 1000.0;
            }

        (void)bfsUs; // suppress unused-variable warning if not printing
        }
    }

    // ── Print benchmark results ────────────────────────────────────────
    std::cout << "\n============================================================\n";
    std::cout << "                   BENCHMARK RESULTS\n";
    std::cout << "============================================================\n\n";

    auto printBlock = [](const std::string& name,
                         double distKm, long long nodes,
                         double ms, bool found)
    {
        std::cout << "[ " << name << " ]\n";
        if (!found) {
            std::cout << "  Status:        No path found\n\n";
        } else {
            std::cout << "  Distance:      " << std::fixed << std::setprecision(2)
                      << distKm << " km\n";
            std::cout << "  Nodes visited: " << nodes << "\n";
            std::cout << "  CPU time:      " << std::fixed << std::setprecision(2)
                      << ms << " ms\n\n";
        }
    };

    printBlock("Dijkstra", dRes.distKm, dRes.nodesVisited,
               dRes.timeUs / 1000.0, dRes.distKm < 1e17);

    printBlock("A* (A-Star)", aRes.distKm, aRes.nodesVisited,
               aRes.timeUs / 1000.0, aRes.distKm < 1e17);

    printBlock("BFS", bfsDistKm, bfsVisited, 0.0, !bfsPath.empty());

    // ── A* vs Dijkstra efficiency summary ─────────────────────────────
    if (dRes.distKm < 1e17 && aRes.distKm < 1e17 && dRes.nodesVisited > 0) {
        double reduction = (1.0 - (double)aRes.nodesVisited / dRes.nodesVisited) * 100.0;
        std::cout << "-> A* visited " << std::fixed << std::setprecision(1)
                  << reduction << "% fewer nodes than Dijkstra.\n";
    }
    std::cout << "\n============================================================\n\n";

    // ── Export all three paths to separate JSON files ──────────────────
    // The frontend loads each file independently and can show all three
    // overlaid on the map for direct visual comparison.
    std::cout << "Exporting paths...\n";

    if (dRes.distKm < 1e17 && !dRes.path.empty())
        exportPath(g, dRes.path, dRes.distKm, dRes.nodesVisited, "dijkstra");
    else
        std::cout << "  -> Dijkstra: no path to export.\n";

    if (aRes.distKm < 1e17 && !aRes.path.empty())
        exportPath(g, aRes.path, aRes.distKm, aRes.nodesVisited, "astar");
    else
        std::cout << "  -> A*: no path to export.\n";

    if (!bfsPath.empty())
        exportPath(g, bfsPath, bfsDistKm, bfsVisited, "bfs");
    else
        std::cout << "  -> BFS: no path to export.\n";

    std::cout << "\nDone. Serve the project folder with:\n";
    std::cout << "  python -m http.server\n";
    std::cout << "Then open http://localhost:8000 in your browser.\n\n";

    return 0;
}
