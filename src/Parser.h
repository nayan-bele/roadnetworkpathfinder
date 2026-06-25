#pragma once
#include "Graph.h"
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

inline long long extractLongLong(const std::string& line, const std::string& attr) {
    std::string key = attr + "=\"";
    size_t pos = line.find(key);
    if (pos == std::string::npos) {
        key = attr + "='";
        pos = line.find(key);
        if (pos == std::string::npos) return 0;
    }
    pos += key.size();
    size_t end = line.find(line[pos - 1], pos);
    if (end == std::string::npos) return 0;
    try {
        return std::stoll(line.substr(pos, end - pos));
    } catch (...) {
        return 0;
    }
}

inline double extractDouble(const std::string& line, const std::string& attr) {
    std::string key = attr + "=\"";
    size_t pos = line.find(key);
    if (pos == std::string::npos) {
        key = attr + "='";
        pos = line.find(key);
        if (pos == std::string::npos) return 0.0;
    }
    pos += key.size();
    size_t end = line.find(line[pos - 1], pos);
    if (end == std::string::npos) return 0.0;
    try {
        return std::stod(line.substr(pos, end - pos));
    } catch (...) {
        return 0.0;
    }
}

inline Graph parseOSM(const std::string& filename) {
    Graph g;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return g;
    }
    std::string line;

    // state for current way
    std::vector<long long> wayNodes;
    bool isHighway = false;

    while (std::getline(file, line)) {
        // parse <node id="..." lat="..." lon="..."
        if (line.find("<node ") != std::string::npos) {
            long long id = extractLongLong(line, "id");
            double lat = extractDouble(line, "lat");
            double lon = extractDouble(line, "lon");
            g.addNode(id, lat, lon);
        }
        // start of a way
        else if (line.find("<way ") != std::string::npos) {
            wayNodes.clear();
            isHighway = false;
        }
        // node reference inside way
        else if (line.find("<nd ref=") != std::string::npos) {
            wayNodes.push_back(extractLongLong(line, "ref"));
        }
        // highway tag — this way is a road
        else if (line.find("k=\"highway\"") != std::string::npos || line.find("k='highway'") != std::string::npos) {
            isHighway = true;
        }
        // end of way
        else if (line.find("</way>") != std::string::npos) {
            if (isHighway) {
                for (size_t i = 0; i + 1 < wayNodes.size(); i++) {
                    g.addEdge(wayNodes[i], wayNodes[i + 1]);
                }
            }
        }
    }
    return g;
}