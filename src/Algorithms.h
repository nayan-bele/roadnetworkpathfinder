#pragma once
#include "Graph.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <iostream>
#include <utility>

//  Dijkstra's Algorithm

inline std::pair<double, std::vector<long long>> dijkstra(
    Graph& g, long long src, long long dst, long long* outNodesVisited = nullptr)
{
    std::unordered_map<long long, double> dist;
    std::unordered_map<long long, long long> prev;
    std::priority_queue<
        std::pair<double, long long>,
        std::vector<std::pair<double, long long>>,
        std::greater<>> pq; 
    for (auto& [id, node] : g.nodes)
        dist[id] = 1e18;

    dist[src] = 0;
    pq.push({0, src});

    long long visitedCount = 0;

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        visitedCount++;

        if (u == dst) break;

        if (d > dist[u]) continue;

        auto it = g.adj.find(u);
        if (it == g.adj.end()) continue; 

        for (auto [v, w] : it->second) {
            double newDist = dist[u] + w;
            if (newDist < dist[v]) {
                dist[v] = newDist;
                prev[v] = u;          
                pq.push({dist[v], v});
            }
        }
    }

    if (outNodesVisited)
        *outNodesVisited = visitedCount;

    if (dist.find(dst) == dist.end() || dist[dst] >= 1e17)
        return {1e18, {}};

    // Reconstruct the path 
    std::vector<long long> path;
    for (long long cur = dst; cur != src; cur = prev[cur]) {
        if (prev.find(cur) == prev.end())
            return {1e18, {}}; 
        path.push_back(cur);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return {dist[dst], path};
}

//  A* (A-Star) Algorithm

inline std::pair<double, std::vector<long long>> astar(
    Graph& g, long long src, long long dst, long long* outNodesVisited = nullptr)
{
    // h(n): estimated remaining distance from node n to the destination.
    auto h = [&](long long n) {
        return haversine(
            g.nodes[n].lat, g.nodes[n].lon,
            g.nodes[dst].lat, g.nodes[dst].lon);
    };

    // gScore[n] = best known real cost from src to n.
    // fScore = gScore + h(n) 
    std::unordered_map<long long, double> gScore;
    std::unordered_map<long long, long long> prev;
    std::priority_queue<
        std::pair<double, long long>,
        std::vector<std::pair<double, long long>>,
        std::greater<>> pq;

    for (auto& [id, node] : g.nodes)
        gScore[id] = 1e18;

    gScore[src] = 0;
    pq.push({h(src), src}); // f = 0 + h(src)

    long long visitedCount = 0;

    while (!pq.empty()) {
        auto [f, u] = pq.top(); pq.pop();
        visitedCount++;

        if (u == dst) break;

        auto it = g.adj.find(u);
        if (it == g.adj.end()) continue;

        for (auto [v, w] : it->second) {
            double tentativeG = gScore[u] + w;
            if (tentativeG < gScore[v]) {
                gScore[v] = tentativeG;
                prev[v] = u;
                pq.push({tentativeG + h(v), v}); 
            }
        }
    }

    if (outNodesVisited)
        *outNodesVisited = visitedCount;

    if (gScore.find(dst) == gScore.end() || gScore[dst] >= 1e17)
        return {1e18, {}};

    std::vector<long long> path;
    for (long long cur = dst; cur != src; cur = prev[cur]) {
        if (prev.find(cur) == prev.end())
            return {1e18, {}};
        path.push_back(cur);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return {gScore[dst], path};
}

//  Breadth-First Search (BFS)

inline std::vector<long long> bfs(Graph& g, long long src, long long dst)
{
    std::unordered_map<long long, long long> prev;
    std::unordered_set<long long> visited;
    std::queue<long long> q;

    q.push(src);
    visited.insert(src);

    bool found = false;
    while (!q.empty()) {
        long long u = q.front(); q.pop();

        if (u == dst) {
            found = true;
            break; 
        }

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

    if (!found && src != dst)
        return {};

    std::vector<long long> path;
    for (long long cur = dst; cur != src; cur = prev[cur]) {
        if (prev.find(cur) == prev.end())
            return {}; 
        path.push_back(cur);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

//  Nearest Node Lookup

inline long long nearestNode(Graph& g, double lat, double lon)
{
    long long best = -1;
    double bestDist = 1e18;

    for (auto& [id, node] : g.nodes) {
        // Skip isolated nodes — you can't route through them.
        if (g.adj.find(id) == g.adj.end()) continue;

        double d = haversine(lat, lon, node.lat, node.lon);
        if (d < bestDist) {
            bestDist = d;
            best = id;
        }
    }

    return best; 
}