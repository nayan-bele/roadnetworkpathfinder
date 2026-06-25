#pragma once // Prevents this header file from being included multiple times in the compiler

#include <unordered_map>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* NODE STRUCT :Represents a single physical intersection or point on a road.*/
struct Node {
    long long id;    // 64-bit unique ID assigned by OpenStreetMap
    double lat, lon; 
};

/* HAVERSINE FORMULA
 * Calculates the real-world distance between two GPS points on a sphere (Earth) in km.
 */
inline double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; 

    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);

    return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

/* GRAPH CLASS */
class Graph {
public:
    // Key : Node ID & Value : The Node object (lat/lon)
    std::unordered_map<long long, Node> nodes;

    // THE ADJACENCY LIST (THE ROADS)
    // Key: Node ID | Value: list of every intersection connected to it.
    // Pair stores: {Connected_Node_ID, Distance_In_Kilometers}
    std::unordered_map<long long, std::vector<std::pair<long long, double>>> adj;

    void addNode(long long id, double lat, double lon) {
        nodes[id] = {id, lat, lon};
    }

    void addEdge(long long u, long long v) {

        // If 'u' or 'v' doesn't exist in our nodes map, we ignore the connection to prevent a crash.
        if (nodes.find(u) == nodes.end() || nodes.find(v) == nodes.end()) {
            return;
        }

        double dist = haversine(
            nodes[u].lat, nodes[u].lon,
            nodes[v].lat, nodes[v].lon);

        adj[u].push_back({v, dist});
        adj[v].push_back({u, dist});
    }
};