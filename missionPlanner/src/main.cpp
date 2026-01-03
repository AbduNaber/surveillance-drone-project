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




// Sadece Header dosyalarını include ediyoruz
#include "gridMap.hpp"
#include "gridMapContext.hpp"
#include "pathFinder.hpp"
#include "common.hpp"



using json = nlohmann::json;


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

    GridMapContext context(map, cellSize);
    
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

    int size = map.getSize();
    int width = size * cellSize;
    int height = size * cellSize;

    // SVG Header
    file << "<svg xmlns='http://www.w3.org/2000/svg' width='" 
         << width << "' height='" << height 
         << "' viewBox='0 0 " << width << " " << height << "'>\n";

    // Draw grid cells
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {

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

    params.path_planning.max_jump_distance = config["path_planning"]["max_jump_distance"].as<int>();
}
void getWaypointsFromUI(gridMap &map, coordinate &startCoord, coordinate &endCoord)
{
    // connect to UI with tcp sock.connect(("127.0.0.1", 5555)) get waypoints
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(5555);
    std::cout << "Waiting for waypoints from UI on port 5555..." << std::endl;
    bind(server_fd, (sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 1);

    int client = accept(server_fd, nullptr, nullptr);

    char buffer[4096] = {};
    read(client, buffer, sizeof(buffer));

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

int main()
{
    // const char *filename = "map.svg";
    // const int gridSize = 200;
    // const double cellSize = 20; // 1 pixel = 1 birim

    appParams params;
    loadParams("params/general_params.yaml", params);

    const char *filename = params.map.map_file.c_str();
    const int gridSize = params.grid.width; 
    const double cellSize = params.grid.cell_size;
    const int maxJumpCells = params.path_planning.max_jump_distance;
    
    // Grid oluştur
    gridMap map(gridSize);

    // SVG'yi yükle ve engelleri/başlangıç/bitiş noktalarını işle
    loadSvgToGrid(filename, map, cellSize);

    std::cout << "Grid loaded from SVG." << std::endl;
    coordinate startCoord{};
    coordinate endCoord{};

    map.inflateObstacles(5); // Inflate obstacles by 5 cells   
    PathFinder pathFinder;
 
    while (1){

   
    // UI'dan waypoint'leri al
    getWaypointsFromUI(map, startCoord, endCoord);
    std::cout << "Start Coord: (" << startCoord.x << ", " << startCoord.y << ")\n";
    std::cout << "End Coord: (" << endCoord.x << ", " << endCoord.y << ")\n";
    std::cout << "Grid Size: " << map.getSize() << std::endl;
    
    // Eğer SVG'den start/end okuyabildiysek yolu bul
   
    
    pathFinder.findPath(map, startCoord, endCoord);
   
    
    // Sonucu dosyaya yaz
    std::ofstream outfile("grid_output.txt");
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    for (int y = 0; y < gridSize; ++y)
    {
        for (int x = 0; x < gridSize; ++x)
        {
            coordinate coord{ x, y };
            if (map.isPath(coord))       outfile << "+"; // Yol en üstte görünsün
            else if (map.isStart(coord)) outfile << "S";
            else if (map.isEnd(coord))   outfile << "E";
            else if (map.isBlocked(coord)) outfile << "#";
            else outfile << ".";
        }
        outfile << '\n';
    }
    outfile.close();
    std::cout << "Grid output written to grid_output.txt" << std::endl;


    pathFinder.printPath();

    IsBlockedFn isBlocked = [&map](int x, int y) { return map.isBlocked(coordinate{x, y}); };
    pathFinder.setPath(pathFinder.smoothPathLOS(pathFinder.getPath(), isBlocked , maxJumpCells));
    pathFinder.updateMap(map);
    
    pathFinder.printPath();
    pathFinder.generateCommands();
    pathFinder.printDroneCommands();
    exportToSVG(map, "output.svg", cellSize);

    if (pathFinder.sendPathToUI() != 0) {
        std::cerr << "Failed to send path to UI." << std::endl;
    } else {
        std::cout << "Path sent to UI successfully." << std::endl;
    }

    map.clearPath();
    }
    return 0;
}