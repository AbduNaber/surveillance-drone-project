#pragma once

#include "gridMap.cpp"
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



typedef boost::mpl::set<
        boost::mpl::pair<tag::element::path, tag::attribute::d>,
        boost::mpl::pair<tag::element::circle, tag::attribute::cx>,
        boost::mpl::pair<tag::element::circle, tag::attribute::cy>,
        boost::mpl::pair<tag::element::circle, tag::attribute::r>,
        tag::attribute::cx,
        tag::attribute::cy,
        tag::attribute::r,
        tag::attribute::id
  >::type processed_attributes_t;


// typedef boost::mpl::insert<traits::shapes_attributes_by_element,
//                         tag::attribute::id>::type processed_attributes_t;


struct circle_policy
    : svgpp::policy::basic_shapes::raw
{
    typedef boost::mpl::set<

                            svgpp::tag::element::circle>
        collect_attributes;
};


class GridMapContext
{

public:
    GridMapContext(gridMap &map, double cellSize)
        : map_(map), cellSize_(cellSize) {}

    void set_circle(double cx, double cy, double r)
    {
        std::cout << "[DEBUG] circle: center=(" << cx << ", " << cy
                  << "), radius=" << r << "\n";

        markCircle(cx, cy, r);

        
    }

    void set_ellipse(double cx, double cy, double rx, double ry)
    {
        std::cout << "[DEBUG] ellipse: center=(" << cx << ", " << cy
                  << "), rx=" << rx << ", ry=" << ry << "\n";
    }

    void set_line(double x1, double y1, double x2, double y2) {}

    void path_move_to(double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "[DEBUG] path_move_to: (" << x << ", " << y << ")\n";
        lastX = x;
        lastY = y;
    }

    void path_line_to(double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "[DEBUG] path_line_to: (" << lastX << ", " << lastY << ") -> ("
                  << x << ", " << y << ")\n";
        markLine(lastX, lastY, x, y);
        lastX = x;
        lastY = y;
    }

    void path_cubic_bezier_to(double x1, double y1, double x2, double y2,
                              double x, double y, tag::coordinate::absolute)
    {
        std::cout << "[DEBUG] path_cubic_bezier_to: (" << lastX << ", " << lastY
                  << ") -> (" << x << ", " << y << ")\n";
    }

    void path_quadratic_bezier_to(double x1, double y1, double x, double y,
                                  tag::coordinate::absolute)
    {

        std::cout << "[DEBUG] path_quadratic_bezier_to: (" << lastX << ", " << lastY
                  << ") -> (" << x << ", " << y << ")\n";
    }

    void path_elliptical_arc_to(double rx, double ry, double x_axis_rotation,
                                bool large_arc_flag, bool sweep_flag, double x,
                                double y, tag::coordinate::absolute)
    {
        std::cout << "[DEBUG] path_elliptical_arc_to: (" << lastX << ", " << lastY
                  << ") -> (" << x << ", " << y << ")\n";
    }

    void path_close_subpath() { std::cout << "[DEBUG] path_close_subpath\n"; }

    void path_exit() { std::cout << "[DEBUG] path_exit\n"; }

    void on_exit_element()
    {
        std::cout << "[DEBUG] on_exit_element\n";
        if (insideCircle)
        {
            
            insideCircle = false;
        }
    }

    void on_enter_element(tag::element::any) {}

    void on_enter_element(tag::element::line)
    {
        std::cout << "[DEBUG] on_enter_element: line\n";
    }
    void on_enter_element(tag::element::circle)
    {
        std::cout << "[DEBUG] on_enter_element: circle\n";
        insideCircle = true;
    }

    void on_enter_element(tag::element::ellipse)
    {
        std::cout << "[DEBUG] on_enter_element: ellipse\n";
    }
    void on_enter_element(tag::element::path)
    {
        std::cout << "[DEBUG] on_enter_element: path\n";
    }

    template <class IRI>
    void set(svgpp::tag::attribute::id, IRI const &value)
    {
        std::cout << "ID: " << value << std::endl;

        std::string str(value.begin(), value.end());

        if (str == "A")
        {
            startOrEnd = 0;
        }
        else if (str == "B")
        {
            startOrEnd = 1;
        }
    }

    void set(svgpp::tag::attribute::cx, double v)
    {
        
        cx_ = v;
    }
    void set(svgpp::tag::attribute::cy, double v)
    {
        
        cy_ = v;
    }
    void set(svgpp::tag::attribute::r, double v)
    {
        
        r_ = v;
    }

private:
    gridMap &map_;
    double cellSize_;
    double lastX = 0, lastY = 0;
    int startOrEnd = -1; // 0 for start, 1 for end
    double cx_, cy_, r_;
    bool insideCircle = false;

    void markLine(double x0, double y0, double x1, double y1)
    {
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

    void markCircle(double cx, double cy, double r)
    {
        std::cout << "[DEBUG] markCircle: center=(" << cx << ", " << cy
                  << "), radius=" << r << "\n";
        int gx0 = int((cx - r) / cellSize_);
        int gy0 = int((cy - r) / cellSize_);
        int gx1 = int((cx + r) / cellSize_);
        int gy1 = int((cy + r) / cellSize_);




        for (int x = gx0; x <= gx1; x++)
            for (int y = gy0; y <= gy1; y++)
            {
                double dx = x * cellSize_ - cx;
                double dy = y * cellSize_ - cy;
                if (dx * dx + dy * dy <= r * r){
                    if(startOrEnd == 0)
                        map_.setStart(coordinate{x, y});
                    else if (startOrEnd == 1)
                        map_.setEnd(coordinate{x, y});
                    else
                        map_.setBlocked(coordinate{x, y});
                }

                    
            }
    }
};



/*TODO
    start ile bitis noktasini daireden ayir
*/