#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include <gtest/gtest.h>
#include <BURST/wall_space.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/numeric_types.hpp>
#include <BURST/geometric_types.hpp>
#include <BURST/kernel_types.hpp>

#include <CGAL/Arr_walk_along_line_point_location.h>

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

        // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
        if (CGAL::orientation_2(points.begin(), points.end()) == CGAL::COLLINEAR) return std::nullopt;

        BURST::geometry::Polygon2D wall_polygon{points.begin(), points.end()};
        return std::optional<TestWallSpace>{TestWallSpace{std::move(wall_polygon)}};
    }
    std::unique_ptr<BURST::geometry::ConfigurationSpace> testConstructConfigurationSpace(BURST::numeric::fscalar robot_radius) const {
        return this->constructConfigurationSpace(robot_radius);
    }
};

/*
 * Define a test fixture containing a valid ConfigurationSpace for testing the MovementModel
 * This should be reused across multiple tests for the MovementModel to keep the geometry consistent
 * This test fixture uses a square configuration geometry
 */
class MovementModelInSquareTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_geometry;
    BURST::geometry::Point2D corner_vertex;
    BURST::geometry::Point2D edge_midpoint;

    void SetUp() override {
        // Construct a TestWallSpace for a square and generate a ConfigurationSpace with a robot radius of 1
        auto wall_geometry = TestWallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{10, 0},
            BURST::geometry::Point2D{10, 10},
            BURST::geometry::Point2D{0, 10}
        });
        // Expect the WallSpace to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace for a regular polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationSpace(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationSpace from WallSpace in test fixture setup";

        // Define a corner and midpoint for use in tests
        this->corner_vertex = BURST::geometry::Point2D{1, 1};
        this->edge_midpoint = BURST::geometry::Point2D{5, 1};
    }
};

/*
 * Define test fixture containing a valid ConfigurationSpace for testing the MovementModel
 * This should be reused across multiple tests for the MovementModel to keep the geometry consistent
 * This test fixture uses a concave configuration geometry
 * In this case, an arrowhead shape
 */
class MovementModelInConcaveTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_geometry;
    BURST::geometry::Point2D concave_vertex;
    BURST::geometry::Point2D edge_midpoint;
    
    void SetUp() override {
        // Construct a TestWallSpace for a concave polygon and generate a ConfigurationSpace with a robot radius of 1
        auto wall_geometry = TestWallSpace::create({
            BURST::geometry::Point2D{0, 20},
            BURST::geometry::Point2D{-20, -20},
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{20, -20}
        });
        // Expect the WallSpace to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace for a simple polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationSpace(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationSpace from WallSpace in test fixture setup";
        

        /*
         * Identify the concave vertex of the configuration geometry for use in tests
         * This can be done using a PointLocation query with a vertical raycast, since the x-value of the concave vertex is known
         */
        // Create a query point way lower than the expected concave vertex
        BURST::CurvedTraits::Point_2 query_origin{0, -100};
        // Create a point location object for the configuration geometry and attach it to the arrangement
        auto arrangement = this->configuration_geometry->set().arrangement();
        CGAL::Arr_walk_along_line_point_location point_location{arrangement};
        // Shoot the ray up
        auto result = point_location.ray_shoot_up(query_origin);
        
        // If the ray hits a vertex, then the vertex is the concave vertex
        if (auto* vertex = std::get_if<decltype(arrangement)::Vertex_const_handle>(&result)) {
            auto x = CGAL::to_double((*vertex)->point().x());
            auto y = CGAL::to_double((*vertex)->point().y());
            this->concave_vertex = BURST::geometry::Point2D{x, y};
        } else if (auto* halfedge = std::get_if<decltype(arrangement)::Halfedge_const_handle>(&result)) {
            auto curve = (*halfedge)->curve();
            if (curve.is_linear()) {
                auto y = CGAL::to_double((-curve.supporting_line().c() - curve.supporting_line().a() * query_origin.x()) / curve.supporting_line().b());
                this->concave_vertex = BURST::geometry::Point2D{CGAL::to_double(query_origin.x()), y};
            } else if (curve.is_circular()) {
                auto center = curve.supporting_circle().center();

                auto cx = CGAL::to_double(center.x());
                auto cy = CGAL::to_double(center.y());
                auto dx = CGAL::to_double(query_origin.x()) - cx;
                auto radius_2 = CGAL::to_double(curve.supporting_circle().squared_radius());
                auto y = cy + std::sqrt(radius_2 - dx * dx);

                this->concave_vertex = BURST::geometry::Point2D{CGAL::to_double(query_origin.x()), y};
            } else {
                FAIL() << "Unexpected curve type in point location result during test fixture setup";
            }
        } else {
            FAIL() << "Unexpected point location result type during test fixture setup";
        }
    }
};

#endif
