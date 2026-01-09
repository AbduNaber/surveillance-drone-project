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
#include <map>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

using IsBlockedFn = std::function<bool(int,int)>;


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

typedef enum {

    // Basic takeoff / landing
    CMD_TAKEOFF,
    CMD_LAND,
    CMD_EMERGENCY,

    // Basic 6-direction movement (distance-based)
    CMD_MOVE_UP,
    CMD_MOVE_DOWN,
    CMD_MOVE_LEFT,
    CMD_MOVE_RIGHT,
    CMD_MOVE_FORWARD,
    CMD_MOVE_BACK,

    // Rotational movement
    CMD_ROTATE_CW,
    CMD_ROTATE_CCW,

    // Hover / stop
    CMD_STOP,

    // Flip maneuvers
    CMD_FLIP_LEFT,
    CMD_FLIP_RIGHT,
    CMD_FLIP_FORWARD,
    CMD_FLIP_BACK,

    // Speed settings
    CMD_SET_SPEED,

    // Unknown / unsupported
    CMD_UNKNOWN
} DroneCommand;


class PathFinder
{
public:

    void findPath(gridMap &map, coordinate start, coordinate end);
    std::vector<coordinate>& getPath() { return path; }
    std::vector<std::map<DroneCommand, std::string>>& getDroneCommands() { return drone_commands; }
    void printPath();
    void generateCommands( appParams &params ); 
    std::vector<coordinate> smoothPathLOS(const std::vector<coordinate>& path,const IsBlockedFn& isBlocked , int maxJumpCells);
    bool lineFree(const coordinate& a, const coordinate& b, const IsBlockedFn& isBlocked);
        
    // for debug porpose delete it
    void setPath(const std::vector<coordinate>& newPath) { path = newPath; }
    void buildDroneCommands( const appParams &params, const std::vector<coordinate>& path, double initialYawRad );
    void updateMap(gridMap &map);
    void printDroneCommands();
    int sendPathToUI();
private:

    std::vector<coordinate> path = {};

    std::vector<std::map<DroneCommand, std::string>> drone_commands; // drone commands and values as string

    static double heuristic(const coordinate &a, const coordinate &b)
    {
        double dx = double(a.x - b.x);
        double dy = double(a.y - b.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    double normalizeAngle(double a) {
        while (a > M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

};




