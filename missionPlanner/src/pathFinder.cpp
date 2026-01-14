#include "pathFinder.hpp"

using json = nlohmann::json;



void PathFinder::findPath(gridMap &map, coordinate start, coordinate end) {
  // trivial cases
  if (start.x == end.x && start.y == end.y)
    return;

  // If start or end are blocked, bail out
  if (map.isBlocked(start) || map.isBlocked(end)) {
    
    std::cout << "[DEBUG] start or end is blocked " << map.isBlocked(start)
              << " " << map.isBlocked(end) << "\n";
    return;
  }

  using PQItem = std::pair<double, coordinate>; // f-score, coordinate

  struct PQComp {
    bool operator()(const PQItem &a, const PQItem &b) const {
      return a.first > b.first;
    }
  };

  std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;

  std::unordered_map<coordinate, double, CoordinateHash, CoordinateEqual>
      gScore;
  std::unordered_map<coordinate, coordinate, CoordinateHash, CoordinateEqual>
      cameFrom;
  std::unordered_set<coordinate, CoordinateHash, CoordinateEqual> closed;

  gScore[start] = 0.0;
  open.push({heuristic(start, end), start});

  // 8-connected moves (4 cardinal + 4 diagonal)
  const int dx[8] = {0, 0, -1, 1, -1, -1, 1, 1};
  const int dy[8] = {-1, 1, 0, 0, -1, 1, -1, 1};
  // Cost: 1.0 for cardinal, sqrt(2) approx 1.414 for diagonal
  const double moveCost[8] = {1.0,           1.0,           1.0,
                              1.0,           1.41421356237, 1.41421356237,
                              1.41421356237, 1.41421356237};

  while (!open.empty()) {
    auto [f, current] = open.top();
    open.pop();

    if (closed.find(current) != closed.end())
      continue;

    // Goal reached
    if (current.x == end.x && current.y == end.y) {
      // Reconstruct path (mark intermediate cells)
      path.clear();
      coordinate node = end;
      while (!(node.x == start.x && node.y == start.y)) {
        path.push_back(node);
        // Don't overwrite start/end markers if you want to preserve them
        if (!(node.x == end.x && node.y == end.y) &&
            !(node.x == start.x && node.y == start.y)) {
          map.setPath(node);
        }

        auto it = cameFrom.find(node);
        if (it == cameFrom.end())
          break;
        node = it->second;
      }

      std::reverse(path.begin(), path.end());
      std::cout << "[DEBUG] Path found from (" << start.x << "," << start.y
                << ") to (" << end.x << "," << end.y << ")\n";
      return;
    }

    closed.insert(current);
    double currentG = gScore[current];

    for (int i = 0; i < 8; ++i) {
      coordinate nb{current.x + dx[i], current.y + dy[i]};

      // map.isBlocked returns true for out-of-bounds too, so use it for bounds
      // check
      if (map.isBlocked(nb))
        continue;

      double tentativeG = currentG + moveCost[i];

      auto it = gScore.find(nb);
      if (it == gScore.end() || tentativeG < it->second) {
        gScore[nb] = tentativeG;
        cameFrom[nb] = current;
        double fscore = tentativeG + heuristic(nb, end);
        open.push({fscore, nb});
      }
    }
  }

  std::cout << "[DEBUG] No path found from (" << start.x << "," << start.y
            << ") to (" << end.x << "," << end.y << ")\n";
}

void PathFinder::printPath() {
  if (path.empty()) {
    std::cout << "[DEBUG] Path is empty.\n";
    return;
  }

  // --- 1) Find bounding box of the path ---
  int minX = path[0].x, maxX = path[0].x;
  int minY = path[0].y, maxY = path[0].y;

  for (auto &p : path) {
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
  for (auto &p : path) {
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
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x)
      std::cout << grid[y][x];
    std::cout << "\n";
  }
}

void PathFinder::generateCommands(appParams &params) {
  drone_commands.clear();
  buildDroneCommands(params,path, 0.0);
}

// it uses Bresenham's line algorithm for that
bool PathFinder::lineFree(const coordinate &a, const coordinate &b,
                          const std::function<bool(int, int)> &isBlocked) {
  int x0 = a.x, y0 = a.y;
  int x1 = b.x, y1 = b.y;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (isBlocked(x0, y0))
      return false;
    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return true;
}

std::vector<coordinate>
PathFinder::smoothPathLOS(const std::vector<coordinate> &path,
                          const std::function<bool(int, int)> &isBlocked,
                          int maxJumpCells) {
  if (path.size() < 3)
    return path;

  std::vector<coordinate> out;
  out.push_back(path.front());

  size_t i = 0;
  while (i < path.size() - 1) {
    size_t best = i + 1;

    for (size_t j = i + 1; j < path.size(); ++j) {
      int dx = path[j].x - path[i].x;
      int dy = path[j].y - path[i].y;

      int dist2 = dx * dx + dy * dy;
      if (dist2 > maxJumpCells * maxJumpCells)
        break;

      if (lineFree(path[i], path[j], isBlocked))
        best = j;
      else
        break;
    }

    out.push_back(path[best]);
    i = best;
  }

  return out;
}

void PathFinder::updateMap(gridMap &map) {
  // clear current path
  map.clearPath();

  for (const auto &p : path) {
    map.setPath(p);
  }
}

void PathFinder::buildDroneCommands(const appParams &params, const std::vector<coordinate> &path,
                                    double initialYawRad ) {
  std::vector<std::map<DroneCommand, std::string>> commands;
  commands.clear();
  commands.push_back(
      std::map<DroneCommand, std::string>{{CMD_SET_SPEED,
                                         std::to_string(static_cast<int>(params.drone.speed))}});
  // Helper lambda to push or merge commands
  auto pushOrMerge = [&](DroneCommand cmd, int value) {
      if (commands.empty()) {
          commands.push_back(std::map<DroneCommand, std::string>{{cmd, std::to_string(value)}});
          return;
      }

      // Check last command
      auto &lastMap = commands.back();
      if (lastMap.empty()) { // Should not happen but safety check
           commands.push_back(std::map<DroneCommand, std::string>{{cmd, std::to_string(value)}});
           return;
      }
      
      auto it = lastMap.begin();
      DroneCommand lastCmd = it->first;
      
      // If same command and mergeable
      if (lastCmd == cmd) {
          bool mergeable = false;
          switch (cmd) {
              case CMD_MOVE_UP:
              case CMD_MOVE_DOWN:
              case CMD_MOVE_LEFT:
              case CMD_MOVE_RIGHT:
              case CMD_MOVE_FORWARD:
              case CMD_MOVE_BACK:
              case CMD_ROTATE_CW:
              case CMD_ROTATE_CCW:
                  mergeable = true;
                  break;
              default:
                  mergeable = false;
          }

          if (mergeable) {
              try {
                  int oldValue = std::stoi(it->second);
                  int newValue = oldValue + value;
                  it->second = std::to_string(newValue);
                  return; // Merged
              } catch (...) {
                  // If conversion fails, just push new
              }
          }
      }

      // Not merged, push new
      commands.push_back(std::map<DroneCommand, std::string>{{cmd, std::to_string(value)}});
  };


  if (path.size() < 2)
    return;

  double currentYaw = initialYawRad;
  commands.push_back(
      std::map<DroneCommand, std::string>{{CMD_TAKEOFF, ""}});

  // optimize push logic for UP command as well? User request specifically mentioned "consecutive".
  // The first move up is usually unique, but using the helper is consistent.
  pushOrMerge(CMD_MOVE_UP, params.drone.takeoff_altitude_cm);
  
  for (size_t i = 0; i + 1 < path.size(); ++i) {

    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;

    double distCm = std::hypot(dx, dy) * params.grid.cell_size;
    int forwardCm = static_cast<int>(std::lround(distCm));


    double targetYaw = std::atan2(dy, dx);
    double deltaYaw = normalizeAngle(targetYaw - currentYaw);
    int turnDeg = static_cast<int>(std::lround(deltaYaw * 180.0 / M_PI));

    // ---- Rotation ----
    if (turnDeg > params.drone.dead_zone_deg || turnDeg < -params.drone.dead_zone_deg){ 
        if (turnDeg > 0) {
            pushOrMerge(CMD_ROTATE_CCW, turnDeg);
      } else if (turnDeg < 0) {
            pushOrMerge(CMD_ROTATE_CW, -turnDeg);
      }
      currentYaw = targetYaw;
    }  

    // ---- Forward ----
    if (forwardCm > 0) {
        pushOrMerge(CMD_MOVE_FORWARD, forwardCm);
    }
  }

  // ---- Comeback Logic ----
  // Traverse the path in reverse order: from end to start
  for (size_t i = path.size() - 1; i > 0; --i) {
      double dx = path[i - 1].x - path[i].x;
      double dy = path[i - 1].y - path[i].y;

      double distCm = std::hypot(dx, dy) * params.grid.cell_size;
      int forwardCm = static_cast<int>(std::lround(distCm));

      double targetYaw = std::atan2(dy, dx);
      double deltaYaw = normalizeAngle(targetYaw - currentYaw);
      int turnDeg = static_cast<int>(std::lround(deltaYaw * 180.0 / M_PI));

      // ---- Rotation ----
      if (turnDeg > params.drone.dead_zone_deg || turnDeg < -params.drone.dead_zone_deg){ 
          if (turnDeg > 0) {
              pushOrMerge(CMD_ROTATE_CCW, turnDeg);
          } else if (turnDeg < 0) {
              pushOrMerge(CMD_ROTATE_CW, -turnDeg);
          }
          currentYaw = targetYaw;
      }

      // ---- Forward ----
      if (forwardCm > 0) {
          pushOrMerge(CMD_MOVE_FORWARD, forwardCm);
      }
  }
  commands.push_back(
      std::map<DroneCommand, std::string>{{CMD_LAND, ""}});
  drone_commands = std::move(commands);
}




void PathFinder::printDroneCommands() {
  for (const auto &cmdMap : drone_commands) {
    for (const auto &pair : cmdMap) {
      DroneCommand cmd = pair.first;
      const std::string &value = pair.second;

      std::cout << "Command: " << cmd;
      if (!value.empty()) {
        std::cout << ", Value: " << value;
      }
      std::cout << "\n";
    }
  }
}

int PathFinder::sendPathToUI()
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5566);          // UI LISTENS here
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return -1;
    }

    json payload;
    payload["type"] = "path";
    payload["path"] = json::array();

    for (const auto& p : path) {
        payload["path"].push_back({
            {"x", p.x},
            {"y", p.y}
        });
    }

    std::string msg = payload.dump();
    send(sock, msg.c_str(), msg.size(), 0);

    close(sock);
    return 0;
}