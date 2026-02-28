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

        // If there's collinear points, the polygon is degenerate, so we can't create a wall space
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
 * This test fixture uses a square configuration space
 */
class MovementModelInSquareTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_space;
    BURST::geometry::Point2D corner_vertex;
    BURST::geometry::Point2D edge_midpoint;

    void SetUp() override {
        // Construct a TestWallSpace for a square and generate a ConfigurationSpace with a robot radius of 1
        auto wall_space = TestWallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{10, 0},
            BURST::geometry::Point2D{10, 10},
            BURST::geometry::Point2D{0, 10}
        });
        // Expect the WallSpace to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a regular polygon in test fixture setup";

        // Construct a configuration space for a robot with radius 1
        this->configuration_space = wall_space->testConstructConfigurationSpace(1.0);
        ASSERT_NE(this->configuration_space, nullptr) << "Failed to construct ConfigurationSpace from WallSpace in test fixture setup";
        ASSERT_EQ(this->configuration_space->orientation(), CGAL::COUNTERCLOCKWISE) << "Expected configuration space to be oriented counterclockwise, but got a different orientation in test fixture setup";

        // Define a corner and midpoint for use in tests
        this->corner_vertex = BURST::geometry::Point2D{1, 1};
        this->edge_midpoint = BURST::geometry::Point2D{5, 1};
    }
};

/*
 * Define test fixture containing a valid ConfigurationSpace for testing the MovementModel
 * This should be reused across multiple tests for the MovementModel to keep the geometry consistent
 * This test fixture uses a concave configuration space
 * In this case, an arrowhead shape
 */
class MovementModelInConcaveTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_space;
    BURST::geometry::Point2D concave_vertex;
    BURST::geometry::Point2D edge_midpoint;

    void find_point_on_configuration_space(BURST::numeric::fscalar origin_x, BURST::geometry::Point2D& result_point) {
        // Create a query point way lower than the expected concave vertex
        BURST::CurvedTraits::Point_2 query_origin{origin_x, -100};
        // Create a point location object for the configuration space and attach it to the arrangement
        auto arrangement = this->configuration_space->set().arrangement();
        CGAL::Arr_walk_along_line_point_location point_location{arrangement};
        // Shoot the ray up
        auto result = point_location.ray_shoot_up(query_origin);
        
        /*
         * If the ray hits a vertex, the result is the concave vertex
         * If the ray hits an edge, the result is the intersection between the ray and the edge, solved using the linear or circular curve equations
         * Everything is lossily converted to double since the traits and fixtures and tests use incompatible number types
         */
        if (auto* vertex = std::get_if<decltype(arrangement)::Vertex_const_handle>(&result)) {
            // Convert coordinates to double and store as the concave vertex
            auto x = (*vertex)->point().x();
            auto y = (*vertex)->point().y();
            result_point = BURST::geometry::Point2D{BURST::numeric::sqrt_to_fscalar(x), BURST::numeric::sqrt_to_fscalar(y)};
        } else if (auto* halfedge = std::get_if<decltype(arrangement)::Halfedge_const_handle>(&result)) {
            auto curve = (*halfedge)->curve();

            if (curve.is_linear()) {
                // Solve for y = (-c - ax) / b using the line equation ax + by + c = 0
                auto y = (-1 * curve.supporting_line().c() - curve.supporting_line().a() * origin_x) / curve.supporting_line().b();
                result_point = BURST::geometry::Point2D{origin_x, y};
            } else if (curve.is_circular()) {
                // Solve for y = cy + sqrt(r^2 - (x - cx)^2) using the circle equation (x - cx)^2 + (y - cy)^2 = r^2
                auto center = curve.supporting_circle().center();

                auto cx = center.x();
                auto cy = center.y();
                auto dx = origin_x - cx;
                auto radius_2 = curve.supporting_circle().squared_radius();
                auto y1 = cy + CGAL::sqrt(radius_2 - dx*dx);
                auto y2 = cy - CGAL::sqrt(radius_2 - dx*dx);
                // Choose the solution that lies on the configuration space
                auto y = this->configuration_space->intersection(BURST::geometry::Point2D{origin_x, y1}).has_value() ? y1 : y2;

                result_point = BURST::geometry::Point2D{origin_x, y};
            } else { // This should never happen since the configuration space should only have linear and circular edges, but handle it anyway
                FAIL() << "Unexpected curve type in point location result during test fixture setup";
            }
        } else { // If the ray hits nothing, then something has gone very wrong since a hit should occur
            FAIL() << "Unexpected point location result type during test fixture setup";
        }
    }
    
    void SetUp() override {
        // Construct a TestWallSpace for a concave polygon and generate a ConfigurationSpace with a robot radius of 1
        auto wall_space = TestWallSpace::create({
            BURST::geometry::Point2D{0, 20},
            BURST::geometry::Point2D{-20, -20},
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{20, -20}
        });
        // Expect the WallSpace to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a simple polygon in test fixture setup";

        // Construct a configuration space for a robot with radius 1
        this->configuration_space = wall_space->testConstructConfigurationSpace(1.0);
        ASSERT_NE(this->configuration_space, nullptr) << "Failed to construct ConfigurationSpace from WallSpace in test fixture setup";

        /*
         * Identify the concave vertex of the configuration space for use in tests
         * This can be done using a PointLocation query with a vertical raycast, since the x-value of the concave vertex is known
         */
        this->find_point_on_configuration_space(0, this->concave_vertex);

        // Identify a point on the edge containing the concave vertex for use in tests using the same raycast method with a different x-value
        this->find_point_on_configuration_space(-10, this->edge_midpoint);
    }
};

#endif
