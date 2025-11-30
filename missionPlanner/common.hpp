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
};
