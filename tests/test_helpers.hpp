#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include <gtest/gtest.h>
#include <BURST/wall_space.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/numeric_types.hpp>
#include <BURST/geometric_types.hpp>
#include <BURST/kernel_types.hpp>

#include <initializer_list>
#include <span>

#include <CGAL/Arr_walk_along_line_point_location.h>

// -- HELPER CLASSES -----------------------------------------------------------

// WallSpace subclass to forward the constructConfigurationSpace method for testing
class TestWallSpace : public BURST::geometry::WallSpace {
private:
    using WallSpace::WallSpace; // Inherit constructors
public:
    // NOTE: These are copy-pasted from WallSpace, so update them if the implementation in WallSpace changes.
    static std::optional<TestWallSpace> create(std::initializer_list<BURST::geometry::Point2D> points) {
        auto wall_polygon_opt = WallSpace::createPolygon(std::span(points));  
        // If nullopt, then the wall polygon was degenerate and we can't create a wall geometry
        return wall_polygon_opt.has_value() ? std::optional<TestWallSpace>{TestWallSpace{wall_polygon_opt.value()}} : std::nullopt;
    }
    static std::optional<TestWallSpace> create(std::initializer_list<BURST::geometry::Point2D> points, std::initializer_list<BURST::geometry::Polygon2D> holes) {
        auto wall_polygon_opt = WallSpace::createPolygon(std::span(points));
        // Degenerate wall polygon, can't create a wall geometry
        if (!wall_polygon_opt) return std::nullopt; 

        // Create a holed polygon to hold the holes and outer boundary
        BURST::geometry::HoledPolygon2D wall_shape{wall_polygon_opt.value()};
        
        // Check that each hole is clockwise-oriented
        for (const BURST::geometry::Polygon2D& hole : holes) {
            // Holes must be oriented clcokwise, so reverse the orientation if not
            BURST::geometry::Polygon2D oriented_hole = hole;
            if (hole.orientation() != CGAL::CLOCKWISE) oriented_hole.reverse_orientation();
            // Add the hole
            wall_shape.add_hole(oriented_hole);
        }

        // Check if the resulting holed polygon is valid, which means:
        // The outer boundary is a simple polygon and counterclockwise-oriented
        // Each hole is a simple polygon and clockwise-oriented
        // Holes are inside the outer boundary and do not intersect with each other
        // Return nullopt if any of these conditions are violated
        return CGAL::is_valid_polygon_with_holes(wall_shape, BURST::LinearTraits{}) ? std::optional<TestWallSpace>{TestWallSpace{wall_shape}} : std::nullopt;

    }
    std::unique_ptr<BURST::geometry::ConfigurationSpace> testConstructConfigurationSpace(BURST::numeric::fscalar robot_radius) const {
        return this->constructConfigurationSpace(robot_radius);
    }
};

#endif
