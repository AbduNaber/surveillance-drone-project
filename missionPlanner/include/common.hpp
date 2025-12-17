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
    } map;

    struct PathPlanningParams
    {
        int max_jump_distance;
    } path_planning;
};