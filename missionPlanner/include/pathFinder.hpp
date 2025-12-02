#pragma once

#include "gridMap.hpp" 
#include "common.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>




struct CoordinateHash {
    std::size_t operator()(const coordinate &c) const noexcept
    {
        return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
    }
};

struct CoordinateEqual {
    bool operator()(const coordinate &a, const coordinate &b) const noexcept
    {
        return a.x == b.x && a.y == b.y;
    }
};



class PathFinder
{
public:

    void findPath(gridMap &map, coordinate start, coordinate end);
    std::vector<coordinate>& getPath() { return path; }

    void printPath();

private:
    std::vector<coordinate> path = {};


    static double heuristic(const coordinate &a, const coordinate &b)
    {
        double dx = double(a.x - b.x);
        double dy = double(a.y - b.y);
        return std::sqrt(dx * dx + dy * dy);
    }

};

