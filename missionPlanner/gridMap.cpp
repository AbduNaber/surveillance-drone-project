#pragma once

#include "common.hpp"

class gridMap
{

public:
    gridMap(int size) : size_(size)
    {
        grid_.resize(size_, std::vector<struct Cell>(size_));
    }

    void setBlocked(const coordinate &c)
    {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].blocked = true;
        }
    }

    bool isBlocked(const coordinate &c) const
    {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            return grid_[y][x].blocked;
        }
        return true; // Out of bounds considered blocked
    }

    bool isStart(const coordinate &c) const {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            return grid_[y][x].isStart;
        }
        return false;
    }

    bool isEnd(const coordinate &c) const {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            return grid_[y][x].isEnd;
        }
        return false;
    }

    bool isPath(const coordinate &c) const {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            return grid_[y][x].isPath;
        }
        return false;
    }

    void setStart(const coordinate &c)
    {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].isStart = true;
        }
    }

    void setEnd(const coordinate &c)
    {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].isEnd = true;
        }
    }

    void setPath(const coordinate &c)
    {
        int x = c.x;
        int y = c.y;
        if (x >= 0 && x < size_ && y >= 0 && y < size_)
        {
            grid_[y][x].isPath = true;
        }
    }
    
    int getSize() const { return size_; }

private:
    int size_;
    std::vector<std::vector<struct Cell>> grid_;
};