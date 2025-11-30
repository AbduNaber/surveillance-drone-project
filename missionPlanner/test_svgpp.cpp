#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <rapidxml_ns/rapidxml_ns.hpp>
#include <svgpp/policy/xml/rapidxml_ns.hpp>
#include <svgpp/svgpp.hpp>

#include <svgpp/policy/basic_shapes.hpp>
#include <svgpp/policy/basic_shapes_events.hpp>

using namespace svgpp;

typedef boost::mpl::set<svgpp::tag::element::svg, svgpp::tag::element::g,
                        svgpp::tag::element::path, svgpp::tag::element::circle,
                        svgpp::tag::element::ellipse,
                        svgpp::tag::element::line>::type processed_elements_t;

typedef boost::mpl::insert<traits::shapes_attributes_by_element,
                           tag::attribute::id>::type processed_attributes_t;

struct collect_attributes_basic_shapes_policy: svgpp::policy::basic_shapes::raw
{
  typedef boost::mpl::set<
    svgpp::tag::element::ellipse,
    svgpp::tag::element::rect,
    svgpp::tag::element::line,
    svgpp::tag::element::circle> collect_attributes; 
};

struct Cell {
  bool blocked = false;
  bool isStart = false;
  bool isEnd = false;
};

class gridMap {

public:
  gridMap(int size) : size_(size) {
    grid_.resize(size_, std::vector<struct Cell>(size_));
  }

  void setBlocked(int x, int y) {
    if (x >= 0 && x < size_ && y >= 0 && y < size_) {
      // std::cout << "[DEBUG] setBlocked: (" << x << ", " << y << ")\n";
      grid_[y][x].blocked = true;
    }
  }

  bool isBlocked(int x, int y) const {
    if (x >= 0 && x < size_ && y >= 0 && y < size_) {

      return grid_[y][x].blocked;
    }
    return true; // Out of bounds considered blocked
  }

  void setStart(int x, int y) {
    if (x >= 0 && x < size_ && y >= 0 && y < size_) {
      grid_[y][x].isStart = true;
    }
  }

  void setEnd(int x, int y) {
    if (x >= 0 && x < size_ && y >= 0 && y < size_) {
      grid_[y][x].isEnd = true;
    }
  }

  int getSize() const { return size_; }

private:
  int size_;
  std::vector<std::vector<struct Cell>> grid_;
};

class GridMapContext {

public:
  GridMapContext(gridMap &map, double cellSize)
      : map_(map), cellSize_(cellSize) {}

  void set_circle(double cx, double cy, double r) {
    std::cout << "[DEBUG] circle: center=(" << cx << ", " << cy
              << "), radius=" << r << "\n";
    markCircle(cx, cy, r);
  }

  void set_ellipse(double cx, double cy, double rx, double ry) {
    std::cout << "[DEBUG] ellipse: center=(" << cx << ", " << cy
              << "), rx=" << rx << ", ry=" << ry << "\n";
  }

  void set_line(double x1,double y1,double x2,double y2 ){

  }

  void path_move_to(double x, double y, svgpp::tag::coordinate::absolute) {
    std::cout << "[DEBUG] path_move_to: (" << x << ", " << y << ")\n";
    lastX = x;
    lastY = y;
  }

  void path_line_to(double x, double y, svgpp::tag::coordinate::absolute) {
    std::cout << "[DEBUG] path_line_to: (" << lastX << ", " << lastY << ") -> ("
              << x << ", " << y << ")\n";
    markLine(lastX, lastY, x, y);
    lastX = x;
    lastY = y;
  }

  void path_cubic_bezier_to(double x1, double y1, double x2, double y2,
                            double x, double y, tag::coordinate::absolute) {
    std::cout << "[DEBUG] path_cubic_bezier_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
  }

  void path_quadratic_bezier_to(double x1, double y1, double x, double y,
                                tag::coordinate::absolute) {

    std::cout << "[DEBUG] path_quadratic_bezier_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
  }

  void path_elliptical_arc_to(double rx, double ry, double x_axis_rotation,
                              bool large_arc_flag, bool sweep_flag, double x,
                              double y, tag::coordinate::absolute) {
    std::cout << "[DEBUG] path_elliptical_arc_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
  }

  void path_close_subpath() { std::cout << "[DEBUG] path_close_subpath\n"; }

  void path_exit() { std::cout << "[DEBUG] path_exit\n"; }

  void on_exit_element() {
    if (insideCircle) {
      markCircle(cx_, cy_, r_);
      insideCircle = false;
    }
  }

  void on_enter_element(tag::element::any) {}

  void on_enter_element(tag::element::line) {
    std::cout << "[DEBUG] on_enter_element: line\n";
  }
  void on_enter_element(tag::element::circle) {
    std::cout << "[DEBUG] on_enter_element: circle\n";
    insideCircle = true;
  }

  void on_enter_element(tag::element::ellipse) {
    std::cout << "[DEBUG] on_enter_element: ellipse\n";
  }
  void on_enter_element(tag::element::path) {
    std::cout << "[DEBUG] on_enter_element: path\n";
  }

  template <class IRI> void set(svgpp::tag::attribute::id, IRI const &value) {
    std::cout << "ID: " << value << std::endl;
    if (value == "A") {
      startOrEnd = 0;
    } else if (value == "B") {
      startOrEnd = 1;
    }
  }

  void set(svgpp::tag::attribute::cx, double v) {
    std::cout << "enters cx" << std::endl;
    cx_ = v;
  }
  void set(svgpp::tag::attribute::cy, double v) {
    std::cout << "enters cy" << std::endl;
    cy_ = v;
  }
  void set(svgpp::tag::attribute::r, double v) {
    std::cout << "enters r" << std::endl;
    r_ = v;
  }

private:
  gridMap &map_;
  double cellSize_;
  double lastX = 0, lastY = 0;
  int startOrEnd;
  double cx_, cy_, r_;
  bool insideCircle = false;

  void markLine(double x0, double y0, double x1, double y1) {
    std::cout << "[DEBUG] markLine: (" << x0 << ", " << y0 << ") -> (" << x1
              << ", " << y1 << ")\n";
    // Bresenham for grid indices
    int gx0 = int(x0 / cellSize_);
    int gy0 = int(y0 / cellSize_);
    int gx1 = int(x1 / cellSize_);
    int gy1 = int(y1 / cellSize_);

    int dx = abs(gx1 - gx0), sx = gx0 < gx1 ? 1 : -1;
    int dy = -abs(gy1 - gy0), sy = gy0 < gy1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
      map_.setBlocked(gx0, gy0);
      if (gx0 == gx1 && gy0 == gy1)
        break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        gx0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        gy0 += sy;
      }
    }
  }

  void markCircle(double cx, double cy, double r) {
    std::cout << "[DEBUG] markCircle: center=(" << cx << ", " << cy
              << "), radius=" << r << "\n";
    int gx0 = int((cx - r) / cellSize_);
    int gy0 = int((cy - r) / cellSize_);
    int gx1 = int((cx + r) / cellSize_);
    int gy1 = int((cy + r) / cellSize_);

    for (int x = gx0; x <= gx1; x++)
      for (int y = gy0; y <= gy1; y++) {
        double dx = x * cellSize_ - cx;
        double dy = y * cellSize_ - cy;
        if (dx * dx + dy * dy <= r * r)
          map_.setBlocked(x, y);
      }
  }

};

void loadSvgToGrid(const char *filename, gridMap &map, double cellSize) {

  std::ifstream file(filename);
  if (!file.is_open()) {
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
      svgpp::basic_shapes_policy<collect_attributes_basic_shapes_policy>
      >::load_document(root, context);
}

int main() {

  const char *filename = "map.svg";
  const int gridSize = 200;
  const double cellSize =
      20.0; // each cell represents 1 unit in SVG coordinates
  gridMap map(gridSize);

  loadSvgToGrid(filename, map, cellSize);

  std::cout << "Grid loaded from SVG." << std::endl;

  // direct to file output
  std::ofstream outfile("grid_output.txt");
  if (!outfile.is_open()) {
    std::cerr << "Failed to open output file." << std::endl;
    return 1;
  }
  for (int y = 0; y < gridSize; ++y) {
    for (int x = 0; x < gridSize; ++x) {
      outfile << (map.isBlocked(x, y) ? '#' : '.');
    }
    outfile << '\n';
  }
  outfile.close();
  std::cout << "Grid output written to grid_output.txt" << std::endl;
  return 0;
}

/*  TODO
 *  
 */
