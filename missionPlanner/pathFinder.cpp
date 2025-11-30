#pragma once

#include "gridMap.cpp"
#include "common.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <iostream>

// A* implementation for gridMap. Marks found path cells with map.setPath(coord).

struct CoordinateHash {
	std::size_t operator()(const coordinate &c) const noexcept
	{
		return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
	}
};

struct CoordinateEqual {
	bool operator()(const coordinate &a, const coordinate &b) const noexcept
	{
		return a.x == b.x && a.y == b.y;
	}
};

static double heuristic(const coordinate &a, const coordinate &b)
{
	double dx = double(a.x - b.x);
	double dy = double(a.y - b.y);
	return std::sqrt(dx * dx + dy * dy);
}

// Public API: findPath
void findPath(gridMap &map, coordinate start, coordinate end)
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
	const double moveCost[8] = {1.0, 1.0, 1.0, 1.0, 1.41421356237, 1.41421356237, 1.41421356237, 1.41421356237};

	while (!open.empty())
	{
		auto [f, current] = open.top();
		open.pop();

		if (closed.find(current) != closed.end())
			continue;

		if (current.x == end.x && current.y == end.y)
		{
			// reconstruct path (mark intermediate cells)
			coordinate node = end;
			while (!(node.x == start.x && node.y == start.y))
			{
				// don't overwrite start/end markers if you want to preserve them
				if (!(node.x == end.x && node.y == end.y) && !(node.x == start.x && node.y == start.y))
					map.setPath(node);

				auto it = cameFrom.find(node);
				if (it == cameFrom.end())
					break;
				node = it->second;
			}
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

