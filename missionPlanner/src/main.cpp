#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// Sadece Header dosyalarını include ediyoruz
#include "gridMap.hpp"
#include "gridMapContext.hpp"
#include "pathFinder.hpp"
#include "common.hpp"

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

int main()
{
    const char *filename = "map.svg";
    const int gridSize = 200;
    const double cellSize = 20; // 1 pixel = 1 birim
    
    // Grid oluştur
    gridMap map(gridSize);

    // SVG'yi yükle ve engelleri/başlangıç/bitiş noktalarını işle
    loadSvgToGrid(filename, map, cellSize);

    std::cout << "Grid loaded from SVG." << std::endl;

    // Haritayı tara ve Start (S) ve End (E) noktalarını bul
    coordinate startCoord = {-1, -1};
    coordinate endCoord = {-1, -1};
    bool startFound = false;
    bool endFound = false;

    // Büyük haritalarda bu döngü maliyetli olabilir ama başlangıç için en güvenli yoldur
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            coordinate c{x, y};
            if (map.isStart(c)) {
                startCoord = c;
                startFound = true;
            } else if (map.isEnd(c)) {
                endCoord = c;
                endFound = true;
            }
            
            if (startFound && endFound) break; 
        }
        if (startFound && endFound) break;
    }

    PathFinder pathFinder;

    // Eğer SVG'den start/end okuyabildiysek yolu bul
    if (startFound && endFound) {
        std::cout << "Start found at: (" << startCoord.x << ", " << startCoord.y << ")\n";
        std::cout << "End found at: (" << endCoord.x << ", " << endCoord.y << ")\n";
        
        pathFinder.findPath(map, startCoord, endCoord);
    } else {
        std::cout << "Uyarı: Start veya End noktası SVG içinde (ID='A' veya ID='B') bulunamadı.\n";
        // İstersen burada manuel bir koordinat da deneyebilirsin:
        // findPath(map, coordinate{700, 3500}, coordinate{3091, 693});
    }

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

    exportToSVG(map, "output.svg", cellSize);

    return 0;
}