#include "gridMap.hpp"

gridMap::gridMap(int width, int height) : width_(width), height_(height)
{
    grid_.resize(height_, std::vector<Cell>(width_));
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

int gridMap::getWidth() const
{
    return width_;
}

int gridMap::getHeight() const
{
    return height_;
}


bool gridMap::isValid(int x, int y) const {
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

void gridMap::clearPath()
{
    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < width_; ++x)
        {
            grid_[y][x].isPath = false;
        }
    }
}


void gridMap::inflateObstacles(int radius) {
    std::vector<coordinate> toBlock;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            coordinate c{x, y};
            if (!isBlocked(c)) continue;

            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    coordinate nb{x + dx, y + dy};
                    if (isValid(nb.x, nb.y)) {
                        toBlock.push_back(nb);
                    }
                }
            }
        }
    }

    for (auto &c : toBlock)
        setBlocked(c);
}

