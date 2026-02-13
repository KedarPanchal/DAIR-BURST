#ifndef TEST_WALLGEOMETRY_CLASS_HPP
#define TEST_WALLGEOMETRY_CLASS_HPP

#include <BURST/wall_geometry.hpp>
#include <BURST/configuration_geometry.hpp>
#include <BURST/types.hpp>

// WallGeometry subclass to forward the constructConfigurationGeometry method for testing
class TestWallGeometry : public BURST::geometry::WallGeometry {
private:
    using WallGeometry::WallGeometry; // Inherit constructors
public:
    static std::optional<TestWallGeometry> create(std::initializer_list<BURST::Point_2> points) noexcept {
        // Copy over logic from WallGeometry::create to construct a TestWallGeometry instead
        // Can't make a polygon with 2 or fewer points
        if (std::distance(points.begin(), points.end()) <= 2) return std::nullopt;

        // Check for self-intersection and overall simplicity of the polygon
        if (!CGAL::is_simple_2(points.begin(), points.end())) return std::nullopt;

        // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
        if (CGAL::orientation_2(points.begin(), points.end()) == CGAL::COLLINEAR) return std::nullopt;

        BURST::Polygon_2 wall_polygon{points.begin(), points.end()};
        return std::optional<TestWallGeometry>{TestWallGeometry{std::move(wall_polygon)}};
    }
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> testConstructConfigurationGeometry(BURST::fscalar robot_radius) const {
        return this->constructConfigurationGeometry(robot_radius);
    }
};

#endif
