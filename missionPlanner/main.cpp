#include "gridMap.cpp"
#include "gridMapContext.cpp"
#include "common.hpp"


/*TODO
 * function: findPath
 * description: Implements A* pathfinding algorithm to find a path from start to end on the grid map.
 * parameters:
 *   - map: Reference to the gridMap object representing the environment.
 *   - startX, startY: Coordinates of the start position.
 *   - endX, endY: Coordinates of the end position.
    * returns: gridMap with the path marked.
*/



void loadSvgToGrid(const char *filename, gridMap &map, double cellSize)
{

    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open SVG file: " << filename << std::endl
                  << std::endl;
        return;
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    std::vector<char> xmlBuffer(content.begin(), content.end());
    xmlBuffer.push_back('\0'); // null-terminated

    rapidxml_ns::xml_document<> doc;
    doc.parse<0>(&xmlBuffer[0]);
    auto root = doc.first_node("svg");

    std::cout << "Loading SVG file: " << filename << std::endl;

    GridMapContext context(map, cellSize);
    svgpp::document_traversal<
        svgpp::processed_elements<processed_elements_t>,
        svgpp::processed_attributes<processed_attributes_t>,
        svgpp::basic_shapes_policy<circle_policy>>::
        load_document(root, context);
}



int main()
{

    const char *filename = "map.svg";
    const int gridSize = 4000;
    const double cellSize =
        1; // each cell represents 1 unit in SVG coordinates
    gridMap map(gridSize);

    loadSvgToGrid(filename, map, cellSize);

    std::cout << "Grid loaded from SVG." << std::endl;

    // direct to file output
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
            if (map.isStart(coord))
                outfile << "S";
            else if (map.isEnd(coord))
                outfile << "E";
            else if (map.isBlocked(coord))
                outfile << "#";
            else if (map.isPath(coord))
                outfile << "*";
            else
                outfile << ".";
        }
        outfile << '\n';
    }
    outfile.close();
    std::cout << "Grid output written to grid_output.txt" << std::endl;
    return 0;
}