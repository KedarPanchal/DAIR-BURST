#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests
#include "test_helpers.hpp"
#include <optional>

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

// Test generating a valid movement with a linear movement model
// This means that the movement should be in a straight line towards the interior of the configuration geometry
 TEST_F(MovementModelInSquareTest, ValidLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the interior at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
}

// Test generating a valid movement with a linear movement model at a corner
 TEST_F(MovementModelInSquareTest, ValidLinearMovementAtCornerInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->corner_vertex;
    // Generate a movement from the corner towards the interior at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
}

// Test generating a valid movement with a linear movement model along the edge
TEST_F(MovementModelInSquareTest, ValidLinearMovementAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the right along the edge at a 0 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, 0, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    }
}

// Test generating an invalid movement with a linear movement model with a point inside but not on the configuration geometry edge
 TEST_F(MovementModelInSquareTest, InvalidInteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin at the center of the configuration geometry
    BURST::Point_2 origin{5, 5};
    // Generate a movement from the interior towards the interior at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point outside the configuration geometry
 TEST_F(MovementModelInSquareTest, InvalidExteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin outside the configuration geometry
    BURST::Point_2 origin{67, 67};
    // Generate a movement from the exterior towards the interior at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point on the edge but a direction pointing outside the configuration geometry
 TEST_F(MovementModelInSquareTest, InvalidLinearMovementPointingOutwardInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a movement from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, -CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating a valid movement with a linear movement model in a concave configuration geometry
TEST_F(MovementModelInConcaveTest, ValidLinearMovementInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the interior at a 90 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
}

// Test generating an valid movement with a linear movement model in a concave configuration geometry at a concave vertex
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the interior at a 90 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
}

// Test generating a valid movement with a linear movement model in a concave configuration geometry along the edge
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAlongEdgeInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the right along the edge at a 45 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_geometry);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    }
}

// Test generating an invalid movement with a linear movement model in a concave configuration geometry at a concave vertex with a direction pointing outside the configuration geometry
TEST_F(MovementModelInConcaveTest, InvalidLinearMovementPointingOutwardAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the exterior at a -90 degree angle
    std::optional<BURST::Point_2> maybe_endpoint = movement_model(origin, -CGAL_PI/2, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}
