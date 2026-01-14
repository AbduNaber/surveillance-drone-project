#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <netinet/in.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

// Sadece Header dosyalarını include ediyoruz
#include "gridMap.hpp"
#include "gridMapContext.hpp"
#include "pathFinder.hpp"
#include "common.hpp"



using json = nlohmann::json;

appParams params;

void loadSvgToGrid(const char *filename, gridMap &map, double cellSize)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open SVG file: " << filename << std::endl;
        return;
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    std::vector<char> xmlBuffer(content.begin(), content.end());
    xmlBuffer.push_back('\0'); // null-terminated

    rapidxml_ns::xml_document<> doc;
    try {
        doc.parse<0>(&xmlBuffer[0]);
    } catch (const std::exception& e) {
        std::cerr << "XML Parse Hatası: " << e.what() << std::endl;
        return;
    }
    
    auto root = doc.first_node("svg");
    if (!root) {
        std::cerr << "Hata: <svg> node'u bulunamadı." << std::endl;
        return;
    }

    std::cout << "Loading SVG file: " << filename << std::endl;

    // --- Dynamic Scale Calculation ---
    double svgWidth = 0.0;
    double svgHeight = 0.0;
    
    // Try to parse viewBox first as it defines the coordinate system
    auto viewBoxAttr = root->first_attribute("viewBox");
    if (viewBoxAttr) {
        std::string viewBoxStr = viewBoxAttr->value();
        std::stringstream ss(viewBoxStr);
        double minX, minY, w, h;
        ss >> minX >> minY >> w >> h;
        svgWidth = w;
        svgHeight = h;
        std::cout << "Parsed viewBox: " << minX << " " << minY << " " << w << " " << h << std::endl;
    } else {
        // Fallback to width/height attributes (simple parsing)
        auto widthAttr = root->first_attribute("width");
        auto heightAttr = root->first_attribute("height");
        if (widthAttr && heightAttr) {
            try {
                svgWidth = std::stod(widthAttr->value());
                svgHeight = std::stod(heightAttr->value());
            } catch (...) {}
        }
    }

    double loadingScale = cellSize; // Default fallback
    if (svgWidth > 0 && svgHeight > 0) {
        double gridW = map.getWidth();
        double gridH = map.getHeight();
        
        double scaleX = svgWidth / gridW;
        double scaleY = svgHeight / gridH;
        
        // Use the larger scale to ensure everything fits (or smaller? usually larger divider means "1 grid cell = N svg units")
        // If we want the whole map to fit in the grid: 
        // 1 grid cell covers (SVG_Dimension / Grid_Dimension) units.
        loadingScale = std::max(scaleX, scaleY);
        
        std::cout << "Detected SVG Dimensions: " << svgWidth << "x" << svgHeight << "\n";
        std::cout << "Grid Dimensions: " << gridW << "x" << gridH << "\n";
        std::cout << "Calculated Loading Scale (Cell Size): " << loadingScale << "\n";
    } else {
        std::cout << "Could not detect SVG dimensions, using configured cell size: " << cellSize << "\n";
    }

    GridMapContext context(map, loadingScale);
    
    using namespace svgpp;
    document_traversal<
        processed_elements<processed_elements_t>,
        processed_attributes<processed_attributes_t>,
        basic_shapes_policy<circle_policy>
    >::load_document(root, context);
}




void exportToSVG(const gridMap &map, const std::string &filename, int cellSize)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open SVG file for writing\n";
        return;
    }

    int width = map.getWidth();
    int height = map.getHeight();
    int imgWidth = width * cellSize;
    int imgHeight = height * cellSize;

    // SVG Header
    file << "<svg xmlns='http://www.w3.org/2000/svg' width='" 
         << imgWidth << "' height='" << imgHeight 
         << "' viewBox='0 0 " << imgWidth << " " << imgHeight << "'>\n";

    // Draw grid cells
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {

            std::string color = "white";

            if (map.isBlocked(coordinate{x, y}))
                color = "red";
            else if (map.isStart(coordinate{x, y}))
                color = "green";
            else if (map.isEnd(coordinate{x, y}))
                color = "blue";
            else if (map.isPath(coordinate{x, y}))
                color = "yellow";

            file << "<rect x='" << x * cellSize 
                 << "' y='" << y * cellSize
                 << "' width='" << cellSize
                 << "' height='" << cellSize
                 << "' fill='" << color << "' stroke='lightgray' stroke-width='0.2'/>\n";
        }
    }

    file << "</svg>";
    file.close();

    std::cout << "SVG exported to: " << filename << "\n";
}

void loadParams(const std::string &paramFile, appParams &params)
{
    std::ifstream file(paramFile);
    if (!file.is_open())
    {
        std::cerr << "Cannot open parameter file: " << paramFile << std::endl;
        return;
    }

    YAML::Node config = YAML::LoadFile(paramFile);

    params.grid.width = config["grid"]["width"].as<int>();
    params.grid.height = config["grid"]["height"].as<int>();
    params.grid.cell_size = config["grid"]["cell_size"].as<double>();

    params.map.map_file = config["map"]["map_file"].as<std::string>(); 
    params.map.resolution = config["map"]["resolution"].as<double>();
    params.map.inflation_radius = config["map"]["inflation_radius"].as<int>();

    params.path_planning.max_jump_distance = config["path_planning"]["max_jump_distance"].as<int>();
    params.drone.speed = config["drone"]["speed"].as<double>();
    params.drone.turn_rate = config["drone"]["turn_rate"].as<double>();
    params.drone.initial_yaw_rad = config["drone"]["initial_yaw_rad"].as<double>();
    params.drone.dead_zone_deg = config["drone"]["dead_zone_deg"].as<double>(); 
    params.drone.takeoff_altitude_cm = config["drone"]["takeoff_altitude_cm"].as<int>();
    
}
void getWaypointsFromUI(gridMap &map, coordinate &startCoord, coordinate &endCoord)
{
    // connect to UI with tcp sock.connect(("127.0.0.1", 5555)) get waypoints
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(5555);
    std::cout << "Waiting for waypoints from UI on port 5555..." << std::endl;
    
    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        close(server_fd);
        return;
    }
    
    if (listen(server_fd, 1) < 0) {
        perror("listen");
        close(server_fd);
        return;
    }

    int client = accept(server_fd, nullptr, nullptr);
    if (client < 0) {
        perror("accept");
        close(server_fd);
        return;
    }
    std::cout << "Client connected!" << std::endl;

    char buffer[4096] = {};
    int bytesRead = read(client, buffer, sizeof(buffer));
    std::cout << "Read " << bytesRead << " bytes from client." << std::endl;
    if (bytesRead <= 0) {
         std::cerr << "Failed to read data or empty." << std::endl;
         close(client);
         close(server_fd);
         return;
    }

    json data = json::parse(buffer);

    for (auto& wp : data["waypoints"]) {
        coordinate c;
        c.x = wp["x"];
        c.y = wp["y"];
    

        std::cout << "Waypoint received: ID=" << wp["id"] << " X=" << c.x << " Y=" << c.y << std::endl;
        if (wp["id"] == "Blocked") {
            map.setBlocked(c);
        } else if (wp["id"] == "A") {
            
            memccpy(&startCoord, &c, sizeof(coordinate), sizeof(coordinate));
            map.setStart(c);
        } else if (wp["id"] == "B") {
            memccpy(&endCoord, &c, sizeof(coordinate), sizeof(coordinate));
            map.setEnd(c);
        }
        
    }

    close(client);
    close(server_fd);
}

void sendCommandsToTello(const std::vector<std::map<DroneCommand, std::string>>& commands)
{
    // Implementation for sending commands to Tello drone with TCP
    // open tcp socket to 5588 port and send commands
    
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5588);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    connect(sock, (sockaddr*)&addr, sizeof(addr));  // 🔒 BLOCKS



    // send as JSON
    json payload;
    payload["type"] = "commands";
    payload["commands"] = json::array();
    for (const auto& cmdMap : commands) {
        json cmdJson;
        for (const auto& pair : cmdMap) {
            DroneCommand cmd = pair.first;
            const std::string &value = pair.second;
            cmdJson["command"] = cmd;
            cmdJson["value"] = value;
        }
        payload["commands"].push_back(cmdJson);
    }



    std::string jsonStr = payload.dump();

    send(sock, jsonStr.c_str(), jsonStr.size(), 0);

    char ack[16];
    recv(sock, ack, sizeof(ack), 0);  // wait for "OK"

    close(sock);

    std::cout << jsonStr << std::endl;
    std::cout << "Commands sent to Tello drone." << std::endl;


}

int main()
{
    // const char *filename = "map.svg";
    // const int gridSize = 200;
    // const double cellSize = 20; // 1 pixel = 1 birim

    
    loadParams("/home/abdu/surveillance_drone_project/missionPlanner/params/general_params.yaml", params);

    const char *filename = params.map.map_file.c_str();
    const int gridWidth = params.grid.width; 
    const int gridHeight = params.grid.height;
    const double cellSize = params.grid.cell_size;
    const int maxJumpCells = params.path_planning.max_jump_distance;
    
    // Grid oluştur
    gridMap map(gridWidth, gridHeight);

    // SVG'yi yükle ve engelleri/başlangıç/bitiş noktalarını işle
    loadSvgToGrid(filename, map, cellSize);

    std::cout << "Grid loaded from SVG." << std::endl;
    coordinate startCoord{};
    coordinate endCoord{};

    map.inflateObstacles(params.map.inflation_radius); // Inflate obstacles by 5 cells   
    PathFinder pathFinder;
 
    while (1){

   
    // UI'dan waypoint'leri al
    getWaypointsFromUI(map, startCoord, endCoord);
    std::cout << "Start Coord: (" << startCoord.x << ", " << startCoord.y << ")\n";
    std::cout << "End Coord: (" << endCoord.x << ", " << endCoord.y << ")\n";
    std::cout << "Grid Size: " << map.getWidth() << "x" << map.getHeight() << std::endl;
    
    // Eğer SVG'den start/end okuyabildiysek yolu bul
   
    
    pathFinder.findPath(map, startCoord, endCoord);
   
    
    // Sonucu dosyaya yaz
    std::ofstream outfile("grid_output.txt");
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    // for (int y = 0; y < gridHeight; ++y)
    // {
    //     for (int x = 0; x < gridWidth; ++x)
    //     {
    //         coordinate coord{ x, y };
    //         if (map.isPath(coord))       outfile << "+"; // Yol en üstte görünsün
    //         else if (map.isStart(coord)) outfile << "S";
    //         else if (map.isEnd(coord))   outfile << "E";
    //         else if (map.isBlocked(coord)) outfile << "#";
    //         else outfile << ".";
    //     }
    //     outfile << '\n';
    // }
    // outfile.close();
    // std::cout << "Grid output written to grid_output.txt" << std::endl;


    pathFinder.printPath();

    IsBlockedFn isBlocked = [&map](int x, int y) { return map.isBlocked(coordinate{x, y}); };
    pathFinder.setPath(pathFinder.smoothPathLOS(pathFinder.getPath(), isBlocked , maxJumpCells));
    pathFinder.updateMap(map);
    
    //pathFinder.printPath();
    pathFinder.generateCommands(params);
    pathFinder.printDroneCommands();
    exportToSVG(map, "output.svg", cellSize);

    if (pathFinder.sendPathToUI() != 0) {
        std::cerr << "Failed to send path to UI." << std::endl;
    } else {
        std::cout << "Path sent to UI successfully." << std::endl;
    }
    sendCommandsToTello(pathFinder.getDroneCommands());

    map.clearPath();
    }
    return 0;
}