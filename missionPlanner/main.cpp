#include "gridMap.cpp"
#include "gridMapContext.cpp"
#include "common.hpp"


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
        svgpp::basic_shapes_policy<collect_attributes_basic_shapes_policy>>::
        load_document(root, context);
}

int main()
{

    const char *filename = "map.svg";
    const int gridSize = 200;
    const double cellSize =
        20.0; // each cell represents 1 unit in SVG coordinates
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
            outfile << (map.isBlocked(x, y) ? '#' : '.');
        }
        outfile << '\n';
    }
    outfile.close();
    std::cout << "Grid output written to grid_output.txt" << std::endl;
    return 0;
}