#include "gridMap.hpp"

gridMap::gridMap(int size) : size_(size)
{
    grid_.resize(size_, std::vector<Cell>(size_));
}

void gridMap::setBlocked(const coordinate &c)
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        grid_[y][x].blocked = true;
    }
}

bool gridMap::isBlocked(const coordinate &c) const
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        return grid_[y][x].blocked;
    }
    return true; // Out of bounds considered blocked
}

bool gridMap::isStart(const coordinate &c) const
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        return grid_[y][x].isStart;
    }
    return false;
}

bool gridMap::isEnd(const coordinate &c) const
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        return grid_[y][x].isEnd;
    }
    return false;
}

bool gridMap::isPath(const coordinate &c) const
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        return grid_[y][x].isPath;
    }
    return false;
}

void gridMap::setStart(const coordinate &c)
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        grid_[y][x].isStart = true;
    }
}

void gridMap::setEnd(const coordinate &c)
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        grid_[y][x].isEnd = true;
    }
}

void gridMap::setPath(const coordinate &c)
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        grid_[y][x].isPath = true;
    }
}

void gridMap::clear(const coordinate &c)
{
    int x = c.x;
    int y = c.y;
    if (isValid(x, y))
    {
        grid_[y][x].isPath = false;
        grid_[y][x].isStart = false;
        grid_[y][x].isEnd = false;
        grid_[y][x].blocked = false;
    }
}

int gridMap::getSize() const
{
    return size_;
}


bool gridMap::isValid(int x, int y) const {
    return (x >= 0 && x < size_ && y >= 0 && y < size_);
}

void gridMap::clearPath()
{
    for (int y = 0; y < size_; ++y)
    {
        for (int x = 0; x < size_; ++x)
        {
            grid_[y][x].isPath = false;
        }
    }
}