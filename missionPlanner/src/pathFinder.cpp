#include "pathFinder.hpp"


void PathFinder::findPath(gridMap &map, coordinate start, coordinate end)
{
    // trivial cases
    if (start.x == end.x && start.y == end.y)
        return;

    // If start or end are blocked, bail out
    if (map.isBlocked(start) || map.isBlocked(end))
    {
        std::cout << "[DEBUG] start or end is blocked\n";
        return;
    }

    using PQItem = std::pair<double, coordinate>; // f-score, coordinate

    struct PQComp {
        bool operator()(const PQItem &a, const PQItem &b) const { return a.first > b.first; }
    };

    std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;

    std::unordered_map<coordinate, double, CoordinateHash, CoordinateEqual> gScore;
    std::unordered_map<coordinate, coordinate, CoordinateHash, CoordinateEqual> cameFrom;
    std::unordered_set<coordinate, CoordinateHash, CoordinateEqual> closed;

    gScore[start] = 0.0;
    open.push({heuristic(start, end), start});

    // 8-connected moves (4 cardinal + 4 diagonal)
    const int dx[8] = {0, 0, -1, 1, -1, -1, 1, 1};
    const int dy[8] = {-1, 1, 0, 0, -1, 1, -1, 1};
    // Cost: 1.0 for cardinal, sqrt(2) approx 1.414 for diagonal
    const double moveCost[8] = {1.0, 1.0, 1.0, 1.0, 1.41421356237, 1.41421356237, 1.41421356237, 1.41421356237};

    while (!open.empty())
    {
        auto [f, current] = open.top();
        open.pop();

        if (closed.find(current) != closed.end())
            continue;

        // Goal reached
        if (current.x == end.x && current.y == end.y)
        {
            // Reconstruct path (mark intermediate cells)
            path.clear();
            coordinate node = end;
            while (!(node.x == start.x && node.y == start.y))
            {
                path.push_back(node);
                // Don't overwrite start/end markers if you want to preserve them
                if (!(node.x == end.x && node.y == end.y) && !(node.x == start.x && node.y == start.y))
                {
                    map.setPath(node);
                }

                auto it = cameFrom.find(node);
                if (it == cameFrom.end())
                    break;
                node = it->second;
            }

            std::reverse(path.begin(), path.end());
            std::cout << "[DEBUG] Path found from (" << start.x << "," << start.y << ") to (" << end.x << "," << end.y << ")\n";
            return;
        }

        closed.insert(current);
        double currentG = gScore[current];

        for (int i = 0; i < 8; ++i)
        {
            coordinate nb{current.x + dx[i], current.y + dy[i]};

            // map.isBlocked returns true for out-of-bounds too, so use it for bounds check
            if (map.isBlocked(nb))
                continue;

            double tentativeG = currentG + moveCost[i];

            auto it = gScore.find(nb);
            if (it == gScore.end() || tentativeG < it->second)
            {
                gScore[nb] = tentativeG;
                cameFrom[nb] = current;
                double fscore = tentativeG + heuristic(nb, end);
                open.push({fscore, nb});
            }
        }
    }

    std::cout << "[DEBUG] No path found from (" << start.x << "," << start.y << ") to (" << end.x << "," << end.y << ")\n";
}

void PathFinder::printPath(){
    if (path.empty())
    {
        std::cout << "[DEBUG] Path is empty.\n";
        return;
    }

    // --- 1) Find bounding box of the path ---
    int minX = path[0].x, maxX = path[0].x;
    int minY = path[0].y, maxY = path[0].y;

    for (auto &p : path)
    {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    int width = maxX - minX + 1;
    int height = maxY - minY + 1;

    // --- 2) Create empty grid (.) ---
    std::vector<std::vector<char>> grid(height, std::vector<char>(width, '.'));

    // --- 3) Mark the path (*) ---
    for (auto &p : path)
    {
        int gx = p.x - minX;
        int gy = p.y - minY;
        grid[gy][gx] = '*';
    }

    // Mark start and end
    coordinate S = path.front();
    coordinate E = path.back();
    grid[S.y - minY][S.x - minX] = 'S';
    grid[E.y - minY][E.x - minX] = 'E';

    // --- 4) Print the grid ---
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
            std::cout << grid[y][x];
        std::cout << "\n";
    }
}