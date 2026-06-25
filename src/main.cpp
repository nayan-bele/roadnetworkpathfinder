#include "Graph.h"
#include "Parser.h"
#include "Algorithms.h"
#include <iostream>

int main() {
    std::cout << "============== Day 1 Routing Test (Nagpur) ==============" << std::endl;
    std::cout << "Parsing map data from data/nagpur.osm..." << std::endl;
    
    Graph g = parseOSM("data/nagpur.osm");
    
    std::cout << "Successfully compiled graph properties:" << std::endl;
    std::cout << " -> Intersections (Nodes) loaded: " << g.nodes.size() << std::endl;
    std::cout << " -> Active driveway systems indexed: " << g.adj.size() << std::endl;

    if (g.nodes.empty()) {
        std::cerr << "Parsing failed or target file empty. Check data/nagpur.osm path placement." << std::endl;
        return 1;
    }

    // Grab a random valid node from the parsed dataset to use as a fallback target
    long long fallbackNode = g.nodes.begin()->first;
    double sampleLat = g.nodes[fallbackNode].lat;
    double sampleLon = g.nodes[fallbackNode].lon;

    std::cout << "\nTesting Route Coordinate Snapping Engine..." << std::endl;
    // We add a tiny offset (+0.005) to simulate an arbitrary user GPS pinpoint click
    long long startNodeId = nearestNode(g, sampleLat + 0.005, sampleLon + 0.005);
    long long endNodeId = nearestNode(g, sampleLat, sampleLon);

    std::cout << " -> Found Start Node Point: " << startNodeId << std::endl;
    std::cout << " -> Found Target End Point: " << endNodeId << std::endl;

    std::cout << "\nRunning Dijkstra Pathfinding Algorithm..." << std::endl;
    auto [shortestDistance, pathRoute] = dijkstra(g, startNodeId, endNodeId);

    if (shortestDistance < 0) {
        std::cout << " -> Result: Path between points is disconnected." << std::endl;
    } else {
        std::cout << " -> Success! Shortest Calculated Route: " << shortestDistance << " km" << std::endl;
        std::cout << " -> Total navigation nodes crossed: " << pathRoute.size() << std::endl;
    }

    return 0;
}