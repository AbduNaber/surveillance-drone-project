#pragma once

#include "common.hpp"

class gridMap
{

public:
    gridMap(int size) : size_(size)
    {
        grid_.resize(size_, std::vector<struct Cell>(size_));
    }

    void setBlocked(int x, int y)
    {
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            // std::cout << "[DEBUG] setBlocked: (" << x << ", " << y << ")\n";
            grid_[y][x].blocked = true;
        }
    }

    bool isBlocked(int x, int y) const
    {
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {

            return grid_[y][x].blocked;
        }
        return true; // Out of bounds considered blocked
    }

    void setStart(int x, int y)
    {
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].isStart = true;
        }
    }

    void setEnd(int x, int y)
    {
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].isEnd = true;
        }
    }

    int getSize() const { return size_; }

private:
    int size_;
    std::vector<std::vector<struct Cell>> grid_;
};