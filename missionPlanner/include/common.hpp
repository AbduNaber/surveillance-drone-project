#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

struct Cell
{
    bool blocked = false;
    bool isStart = false;
    bool isEnd = false;
    bool isPath = false ;
};

struct coordinate
{
    int x;
    int y;
};

struct appParams
{
    struct GridParams
    {
        int width;
        int height;
        double cell_size;
    } grid;

    struct MapParams
    {
        std::string map_file;
        double resolution; // meters per pixel
        int inflation_radius; // in cells
    } map;

    struct PathPlanningParams
    {
        int max_jump_distance;
    } path_planning;

    struct DroneParams
    {
        double speed;        // meters per second
        double turn_rate;    // degrees per second
        double initial_yaw_rad; // initial yaw in radians
        double dead_zone_deg;   // degrees
        int takeoff_altitude_cm; // takeoff altitude in centimeters
    } drone;
};