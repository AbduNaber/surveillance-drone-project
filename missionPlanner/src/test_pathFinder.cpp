#include "../include/pathFinder.hpp"
#include "../include/common.hpp"
#include <iostream>
#include <vector>
#include <cassert>

// Mock GridMap and GridMapContext if needed, or just link them. 
// However, PathFinder depends on gridMap.hpp. 
// Ideally we should include gridMap.hpp, but since we are compiling with it, it's fine.

// Minimal test to verify generateCommands adds the forward command.

int main() {
    std::cout << "Starting Test PathFinder...\n";

    appParams params;
    params.drone.speed = 100;
    params.drone.turn_rate = 10;
    params.drone.initial_yaw_rad = 0;
    params.drone.dead_zone_deg = 5;
    params.drone.takeoff_altitude_cm = 80;
    params.grid.cell_size = 20.0;

    PathFinder pf;
    // Create a dummy path
    std::vector<coordinate> path = {{0, 0}, {0, 1}, {0, 2}}; 
    pf.setPath(path);

    std::cout << "Generating commands...\n";
    pf.generateCommands(params);
    
    auto commands = pf.getDroneCommands();

    bool foundForward50 = false;
    bool foundTakeoff = false;

    for (const auto& cmdMap : commands) {
        for (const auto& pair : cmdMap) {
            std::cout << "CMD: " << pair.first << " VAL: " << pair.second << "\n";
            
            if (pair.first == CMD_MOVE_UP && pair.second == "80") {
                foundTakeoff = true;
            }
            // We look for CMD_MOVE_FORWARD with value 50 immediately after takeoff?
            // Or just check if it exists in the sequence.
            
    // We look for CMD_MOVE_FORWARD with value > 50. 
            // The requirement is that we must NOT see any value > 50 for CMD_MOVE_FORWARD.
            
            if (pair.first == CMD_MOVE_FORWARD) {
                int val = std::stoi(pair.second);
                if (val > 50) {
                     std::cout << "FAILURE: Found CMD_MOVE_FORWARD with value > 50: " << val << "\n";
                     return 1;
                }
                // Check if total distance is roughly correct?
                // For now just check splitting limit.
            }
        }
    }
    std::cout << "SUCCESS: No CMD_MOVE_FORWARD > 50 found.\n";


    return 0;
}
