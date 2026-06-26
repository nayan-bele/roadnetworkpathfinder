#pragma once
#include "Algorithms.h"
#include <chrono>
#include <vector>
#include <iostream>
#include <iomanip>

struct BenchResult {
    double distKm;
    long long nodesVisited;
    long long timeUs;
    std::vector<long long> path;
};

inline BenchResult benchDijkstra(Graph& g, long long src, long long dst) {
    long long visited = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto [dist, path] = dijkstra(g, src, dst, &visited);
    auto end = std::chrono::high_resolution_clock::now();
    long long us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    return {dist, visited, us, path};
}

inline BenchResult benchAStar(Graph& g, long long src, long long dst) {
    long long visited = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto [dist, path] = astar(g, src, dst, &visited);
    auto end = std::chrono::high_resolution_clock::now();
    long long us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    return {dist, visited, us, path};
}

inline void printComparison(BenchResult& d, BenchResult& a) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nDijkstra: "
              << d.distKm << "km | "
              << d.nodesVisited << " nodes | "
              << d.timeUs << "us\n";
    std::cout << "A*:       "
              << a.distKm << "km | "
              << a.nodesVisited << " nodes | "
              << a.timeUs << "us\n";
    if (a.nodesVisited > 0) {
        std::cout << "A* speedup: "
                  << (double)d.nodesVisited / a.nodesVisited
                  << "x fewer nodes visited\n";
    } else {
        std::cout << "A* speedup: N/A\n";
    }
}