#pragma once

#include <vector>
#include "common.hpp"

class gridMap
{
public:
    // Constructor
    gridMap(int width, int height);

    // Setters
    void setBlocked(const coordinate &c);
    void setStart(const coordinate &c);
    void setEnd(const coordinate &c);
    void setPath(const coordinate &c);
    void clear(const coordinate &c);
    void clearPath();
    // Getters
    bool isBlocked(const coordinate &c) const;
    bool isStart(const coordinate &c) const;
    bool isEnd(const coordinate &c) const;
    bool isPath(const coordinate &c) const;
    void inflateObstacles(int radius);
    int getWidth() const;
    int getHeight() const;

private:
    int width_;
    int height_;
    // Assuming 'struct Cell' is defined in common.hpp
    std::vector<std::vector<Cell>> grid_; 

    bool isValid(int x, int y) const ;
};