#ifndef TEST_WALLGEOMETRY_CLASS_HPP
#define TEST_WALLGEOMETRY_CLASS_HPP

#include <gtest/gtest.h>
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

// Define a test fixture containing a valid ConfigurationGeometry for testing the MovementModel
// This should be reused across multiple tests for the MovementModel to keep the geometry consistent
// This test fixture uses a square configuration geometry
class MovementModelInSquareTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_geometry;
    BURST::Point_2 corner_vertex;
    BURST::Point_2 edge_midpoint;

    void SetUp() override {
        // Construct a TestWallGeometry for a square and generate a ConfigurationGeometry with a robot radius of 1
        auto wall_geometry = TestWallGeometry::create({
            BURST::Point_2{0, 0},
            BURST::Point_2{10, 0},
            BURST::Point_2{10, 10},
            BURST::Point_2{0, 10}
        });
        // Expect the WallGeometry to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationGeometry from WallGeometry in test fixture setup";

        // Define a corner and midpoint for use in tests
        this->corner_vertex = BURST::Point_2{1, 1};
        this->edge_midpoint = BURST::Point_2{5, 1};
    }
};

// Define test fixture containing a valid ConfigurationGeometry for testing the MovementModel
// This should be reused across multiple tests for the MovementModel to keep the geometry consistent
// This test fixture uses a concave configuration geometry
// In this case, an arrowhead shape
class MovementModelInConcaveTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_geometry;
    BURST::Point_2 concave_vertex;
    BURST::Point_2 edge_midpoint;
    
    void SetUp() override {
        // Construct a TestWallGeometry for a concave polygon and generate a ConfigurationGeometry with a robot radius of 1
        auto wall_geometry = TestWallGeometry::create({
            BURST::Point_2{0, 20},
            BURST::Point_2{-20, -20},
            BURST::Point_2{0, 0},
            BURST::Point_2{20, -20}
        });
        // Expect the WallGeometry to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a simple polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationGeometry from WallGeometry in test fixture setup";
        

        // Identify the concave vertex of the configuration geometry for use in tests
        for (auto vertex_it = this->configuration_geometry->vertex_begin(); vertex_it != this->configuration_geometry->vertex_end(); vertex_it++) {
            if (vertex_it->x() == 0 && vertex_it->y() < 2) {
                this->concave_vertex = *vertex_it;
                break;
            }
        }
        
        // Find an edge of the configuration geometry that contains the concave vertex and identify its midpoint for use in tests
        for (auto edge_it = this->configuration_geometry->edge_begin(); edge_it != this->configuration_geometry->edge_end(); edge_it++) {
            if (edge_it->target().x() == 0 && edge_it->target().y() < 2) {
                this->edge_midpoint = CGAL::midpoint(edge_it->source(), edge_it->target());
                break;
            }
        }
    }
};

#endif
