#include "gridMapContext.hpp"

// We can use the namespace here for cleaner implementation code
using namespace svgpp;

GridMapContext::GridMapContext(gridMap &map, double cellSize)
    : map_(map), cellSize_(cellSize) 
{
}

// --- Shape Handlers ---

void GridMapContext::set_circle(double cx, double cy, double r)
{
    std::cout << "[DEBUG] circle: center=(" << cx << ", " << cy
              << "), radius=" << r << "\n";
    markCircle(cx, cy, r);
}

void GridMapContext::set_ellipse(double cx, double cy, double rx, double ry)
{
    std::cout << "[DEBUG] ellipse: center=(" << cx << ", " << cy
              << "), rx=" << rx << ", ry=" << ry << "\n";
}

void GridMapContext::set_line(double x1, double y1, double x2, double y2) 
{
    // Implementation was empty in original
}

// --- Path Handlers ---

void GridMapContext::path_move_to(double x, double y, tag::coordinate::absolute)
{
    std::cout << "[DEBUG] path_move_to: (" << x << ", " << y << ")\n";
    lastX = x;
    lastY = y;
}

void GridMapContext::path_line_to(double x, double y, tag::coordinate::absolute)
{
    std::cout << "[DEBUG] path_line_to: (" << lastX << ", " << lastY << ") -> ("
              << x << ", " << y << ")\n";
    markLine(lastX, lastY, x, y);
    lastX = x;
    lastY = y;
}

void GridMapContext::path_cubic_bezier_to(double x1, double y1, double x2, double y2,
                          double x, double y, tag::coordinate::absolute)
{
    std::cout << "[DEBUG] path_cubic_bezier_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
}

void GridMapContext::path_quadratic_bezier_to(double x1, double y1, double x, double y,
                              tag::coordinate::absolute)
{
    std::cout << "[DEBUG] path_quadratic_bezier_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
}

void GridMapContext::path_elliptical_arc_to(double rx, double ry, double x_axis_rotation,
                            bool large_arc_flag, bool sweep_flag, double x,
                            double y, tag::coordinate::absolute)
{
    std::cout << "[DEBUG] path_elliptical_arc_to: (" << lastX << ", " << lastY
              << ") -> (" << x << ", " << y << ")\n";
}

void GridMapContext::path_close_subpath() 
{ 
    std::cout << "[DEBUG] path_close_subpath\n"; 
}

void GridMapContext::path_exit() 
{ 
    std::cout << "[DEBUG] path_exit\n"; 
}

// --- Element Events ---

void GridMapContext::on_exit_element()
{
    std::cout << "[DEBUG] on_exit_element\n";
    if (insideCircle)
    {
        insideCircle = false;
    }
}

void GridMapContext::on_enter_element(tag::element::any) {}

void GridMapContext::on_enter_element(tag::element::line)
{
    std::cout << "[DEBUG] on_enter_element: line\n";
}

void GridMapContext::on_enter_element(tag::element::circle)
{
    std::cout << "[DEBUG] on_enter_element: circle\n";
    insideCircle = true;
}

void GridMapContext::on_enter_element(tag::element::ellipse)
{
    std::cout << "[DEBUG] on_enter_element: ellipse\n";
}

void GridMapContext::on_enter_element(tag::element::path)
{
    std::cout << "[DEBUG] on_enter_element: path\n";
}

// --- Attribute Setters ---

void GridMapContext::set(tag::attribute::cx, double v)
{
    cx_ = v;
}

void GridMapContext::set(tag::attribute::cy, double v)
{
    cy_ = v;
}

void GridMapContext::set(tag::attribute::r, double v)
{
    r_ = v;
}

// --- Helper Functions ---

void GridMapContext::markLine(double x0, double y0, double x1, double y1)
{
    std::cout << "[DEBUG] markLine: (" << x0 << ", " << y0 << ") -> (" << x1
              << ", " << y1 << ")\n";
    
    // Bresenham for grid indices
    int gx0 = int(x0 / cellSize_);
    int gy0 = int(y0 / cellSize_);
    int gx1 = int(x1 / cellSize_);
    int gy1 = int(y1 / cellSize_);

    int dx = std::abs(gx1 - gx0), sx = gx0 < gx1 ? 1 : -1;
    int dy = -std::abs(gy1 - gy0), sy = gy0 < gy1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
        map_.setBlocked(coordinate{gx0, gy0});
        if (gx0 == gx1 && gy0 == gy1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            gx0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            gy0 += sy;
        }
    }
}

void GridMapContext::markCircle(double cx, double cy, double r)
{
    std::cout << "[DEBUG] markCircle: center=(" << cx << ", " << cy
              << "), radius=" << r << "\n";
    
    int gx0 = int((cx - r) / cellSize_);
    int gy0 = int((cy - r) / cellSize_);
    int gx1 = int((cx + r) / cellSize_);
    int gy1 = int((cy + r) / cellSize_);

    for (int x = gx0; x <= gx1; x++)
    {
        for (int y = gy0; y <= gy1; y++)
        {
            double dx = x * cellSize_ - cx;
            double dy = y * cellSize_ - cy;
            
            if (dx * dx + dy * dy <= r * r)
            {
                if (startOrEnd == 0)
                    map_.setStart(coordinate{x, y});
                else if (startOrEnd == 1)
                    map_.setEnd(coordinate{x, y});
                else
                    map_.setBlocked(coordinate{x, y});
            }
        }
    }
}