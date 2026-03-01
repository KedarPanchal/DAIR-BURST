#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include <gtest/gtest.h>
#include <BURST/wall_space.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/numeric_types.hpp>
#include <BURST/geometric_types.hpp>
#include <BURST/kernel_types.hpp>

#include <CGAL/Arr_walk_along_line_point_location.h>

// -- HELPER CLASSES -----------------------------------------------------------

// WallSpace subclass to forward the constructConfigurationSpace method for testing
class TestWallSpace : public BURST::geometry::WallSpace {
private:
    using WallSpace::WallSpace; // Inherit constructors
public:
    static std::optional<TestWallSpace> create(std::initializer_list<BURST::geometry::Point2D> points) noexcept {
        // Copy over logic from WallSpace::create to construct a TestWallSpace instead
        // Can't make a polygon with 2 or fewer points
        if (std::distance(points.begin(), points.end()) <= 2) return std::nullopt;

        // Check for self-intersection and overall simplicity of the polygon
        if (!CGAL::is_simple_2(points.begin(), points.end())) return std::nullopt;

        // If there's collinear points, the polygon is degenerate, so we can't create a wall space
        if (CGAL::orientation_2(points.begin(), points.end()) == CGAL::COLLINEAR) return std::nullopt;

        BURST::geometry::Polygon2D wall_polygon{points.begin(), points.end()};
        return std::optional<TestWallSpace>{TestWallSpace{std::move(wall_polygon)}};
    }
    std::unique_ptr<BURST::geometry::ConfigurationSpace> testConstructConfigurationSpace(BURST::numeric::fscalar robot_radius) const {
        return this->constructConfigurationSpace(robot_radius);
    }
};

#endif
