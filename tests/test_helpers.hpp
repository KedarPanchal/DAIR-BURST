#ifndef TEST_WALLGEOMETRY_CLASS_HPP
#define TEST_WALLGEOMETRY_CLASS_HPP

#include <gtest/gtest.h>
#include <BURST/wall_geometry.hpp>
#include <BURST/configuration_geometry.hpp>
#include <BURST/types.hpp>
#include <BURST/functions.hpp>
#include <CGAL/Arr_walk_along_line_point_location.h>

#include <initializer_list>

// WallGeometry subclass to forward the constructConfigurationGeometry method for testing
class TestWallGeometry : public BURST::geometry::WallGeometry {
private:
    using WallGeometry::WallGeometry; // Inherit constructors
public:
    static std::optional<TestWallGeometry> create(std::initializer_list<BURST::Point2D<BURST::RationalKernel>> points) noexcept {
        // Copy over logic from WallGeometry::create to construct a TestWallGeometry instead
        // Can't make a polygon with 2 or fewer points
        if (std::distance(points.begin(), points.end()) <= 2) return std::nullopt;

        // Check for self-intersection and overall simplicity of the polygon
        if (!CGAL::is_simple_2(points.begin(), points.end())) return std::nullopt;

        // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
        if (CGAL::orientation_2(points.begin(), points.end()) == CGAL::COLLINEAR) return std::nullopt;

        BURST::Polygon2D wall_polygon{points.begin(), points.end()};
        return std::optional<TestWallGeometry>{TestWallGeometry{std::move(wall_polygon)}};
    }
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> testConstructConfigurationGeometry(BURST::rscalar robot_radius) const {
        return this->constructConfigurationGeometry(robot_radius);
    }
};

// Define a test fixture containing a valid ConfigurationGeometry for testing the MovementModel
// This should be reused across multiple tests for the MovementModel to keep the geometry consistent
// This test fixture uses a square configuration geometry
class MovementModelInSquareTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_geometry;
    BURST::Point2D<BURST::AlgebraicKernel> corner_vertex;
    BURST::Point2D<BURST::AlgebraicKernel> edge_midpoint;

    void SetUp() override {
        // Construct a TestWallGeometry for a square and generate a ConfigurationGeometry with a robot radius of 1
        auto wall_geometry = TestWallGeometry::create({
            BURST::Point2D<BURST::RationalKernel>{0, 0},
            BURST::Point2D<BURST::RationalKernel>{10, 0},
            BURST::Point2D<BURST::RationalKernel>{10, 10},
            BURST::Point2D<BURST::RationalKernel>{0, 10}
        });
        // Expect the WallGeometry to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationGeometry from WallGeometry in test fixture setup";

        // Define a corner and midpoint for use in tests
        this->corner_vertex = BURST::Point2D<BURST::AlgebraicKernel>{1, 1};
        this->edge_midpoint = BURST::Point2D<BURST::AlgebraicKernel>{5, 1};
    }
};

// Define test fixture containing a valid ConfigurationGeometry for testing the MovementModel
// This should be reused across multiple tests for the MovementModel to keep the geometry consistent
// This test fixture uses a concave configuration geometry
// In this case, an arrowhead shape
class MovementModelInConcaveTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_geometry;
    BURST::Point2D<BURST::AlgebraicKernel> concave_vertex;
    BURST::Point2D<BURST::AlgebraicKernel> edge_midpoint;
    
    void SetUp() override {
        // Construct a TestWallGeometry for a concave polygon and generate a ConfigurationGeometry with a robot radius of 1
        auto wall_geometry = TestWallGeometry::create({
            BURST::Point2D<BURST::RationalKernel>{0, 20},
            BURST::Point2D<BURST::RationalKernel>{-20, -20},
            BURST::Point2D<BURST::RationalKernel>{0, 0},
            BURST::Point2D<BURST::RationalKernel>{20, -20}
        });
        // Expect the WallGeometry to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a simple polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1.0);
        ASSERT_NE(this->configuration_geometry, nullptr) << "Failed to construct ConfigurationGeometry from WallGeometry in test fixture setup";
        
        // Define a concave vertex for use in tests
        // Find this by performing a raycast up from a really low y value
        BURST::ConicTraits traits;
        CGAL::Arr_walk_along_line_point_location<BURST::ClosedCurve2D> point_location{*this->configuration_geometry};
        BURST::Point2D<BURST::AlgebraicKernel> ray_origin{0, -100};
        auto ray_result = point_location.ray_shoot_up(ray_origin);
        if (auto* edge = std::get_if<BURST::halfedge_iterator<BURST::ClosedCurve2D>>(&ray_result)) {
            // Find intersections of some vertical line with the edge to find the exact point of intersection, which is the concave vertex
            std::list<std::variant<std::pair<BURST::ConicTraits::Point_2, unsigned int>, BURST::ConicTraits::X_monotone_curve_2>> intersections;
            auto intersect = traits.intersect_2_object();
            auto vertical_ray = traits.construct_x_monotone_curve_2_object()(ray_origin, BURST::Point2D<BURST::AlgebraicKernel>{0, 3});
            intersect((*edge)->curve(), vertical_ray, std::back_inserter(intersections));

            // Loop to find the intersection point, which is the concave vertex
            for (const auto& intersection : intersections) {
                if (auto* pt = std::get_if<std::pair<BURST::ConicTraits::Point_2, unsigned int>>(&intersection)) {
                    this->concave_vertex = pt->first;
                    break;
                }
            }
        } else {
            FAIL() << "Failed to find an edge for ray shooting in test fixture setup";
        }

        // Define a midpoint for use in tests
        // We'll just use the midpoint of the edge containing the concave vertex, which we can find by iterating through the edges and finding the one that contains the concave vertex
        for (auto edge_it = this->configuration_geometry->edge_begin(); edge_it != this->configuration_geometry->edge_end(); ++edge_it) {
            if (BURST::curved_has_point(edge_it, this->concave_vertex)) {
                auto source = edge_it->source()->point();
                auto target = edge_it->target()->point();
                this->edge_midpoint = CGAL::midpoint(source, target);
                break;
            }
        }
    }
};

#endif
