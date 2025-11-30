#pragma once

// Standard and Library Includes
#include <vector>
#include <string>
#include <iostream>
#include <cmath> // for abs

// Boost and SVGPP Includes
#include <rapidxml_ns/rapidxml_ns.hpp>
#include <svgpp/policy/xml/rapidxml_ns.hpp>
#include <svgpp/svgpp.hpp>
#include <svgpp/policy/basic_shapes.hpp>
#include <svgpp/policy/basic_shapes_events.hpp>

// Project Includes
#include "common.hpp"
#include "gridMap.hpp" // Changed from .cpp to .hpp

// Define types required by SVGPP. 
// Fully qualified to avoid "using namespace" in a header file.
typedef boost::mpl::set<
    svgpp::tag::element::svg, 
    svgpp::tag::element::g,
    svgpp::tag::element::path, 
    svgpp::tag::element::circle,
    svgpp::tag::element::ellipse,
    svgpp::tag::element::line
>::type processed_elements_t;

typedef boost::mpl::set<
    boost::mpl::pair<svgpp::tag::element::path, svgpp::tag::attribute::d>,
    boost::mpl::pair<svgpp::tag::element::circle, svgpp::tag::attribute::cx>,
    boost::mpl::pair<svgpp::tag::element::circle, svgpp::tag::attribute::cy>,
    boost::mpl::pair<svgpp::tag::element::circle, svgpp::tag::attribute::r>,
    svgpp::tag::attribute::cx,
    svgpp::tag::attribute::cy,
    svgpp::tag::attribute::r,
    svgpp::tag::attribute::id
>::type processed_attributes_t;


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
    GridMapContext(gridMap &map, double cellSize);

    // --- Shape Handlers ---
    void set_circle(double cx, double cy, double r);
    void set_ellipse(double cx, double cy, double rx, double ry);
    void set_line(double x1, double y1, double x2, double y2);

    // --- Path Handlers ---
    void path_move_to(double x, double y, svgpp::tag::coordinate::absolute);
    void path_line_to(double x, double y, svgpp::tag::coordinate::absolute);
    void path_cubic_bezier_to(double x1, double y1, double x2, double y2,
                              double x, double y, svgpp::tag::coordinate::absolute);
    void path_quadratic_bezier_to(double x1, double y1, double x, double y,
                                  svgpp::tag::coordinate::absolute);
    void path_elliptical_arc_to(double rx, double ry, double x_axis_rotation,
                                bool large_arc_flag, bool sweep_flag, double x,
                                double y, svgpp::tag::coordinate::absolute);
    void path_close_subpath();
    void path_exit();

    // --- Element Events ---
    void on_exit_element();
    void on_enter_element(svgpp::tag::element::any);
    void on_enter_element(svgpp::tag::element::line);
    void on_enter_element(svgpp::tag::element::circle);
    void on_enter_element(svgpp::tag::element::ellipse);
    void on_enter_element(svgpp::tag::element::path);

    // --- Attribute Setters ---

    // Template method must remain in header
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

    void set(svgpp::tag::attribute::cx, double v);
    void set(svgpp::tag::attribute::cy, double v);
    void set(svgpp::tag::attribute::r, double v);

private:
    gridMap &map_;
    double cellSize_;
    double lastX = 0, lastY = 0;
    int startOrEnd = -1; // 0 for start, 1 for end
    double cx_, cy_, r_;
    bool insideCircle = false;

    // Helper functions
    void markLine(double x0, double y0, double x1, double y1);
    void markCircle(double cx, double cy, double r);
};