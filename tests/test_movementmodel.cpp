#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests
#include "test_helpers.hpp"
#include <optional>

// -- TEST FIXTURE SETUP -------------------------------------------------------

// Define a test fixture containing a valid ConfigurationSpace for testing the MovementModel
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


// -- MOVEMENTMODEL FUNCTOR TESTS ----------------------------------------------

// Test generating a valid movement with a linear movement model in a square
// This means that the movement should be in a straight line towards the interior of the configuration space
TEST_F(MovementModelInSquareTest, ValidLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration space as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating a valid movement with a linear movement model at a corner in a square
TEST_F(MovementModelInSquareTest, ValidLinearMovementAtCornerInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a movement from the corner towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating a valid movement with a linear movement model along the edge in a square
TEST_F(MovementModelInSquareTest, ValidLinearMovementAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration space as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, 0, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating a valid movement with a linear movement model at a corner and along the edge in a square
TEST_F(MovementModelInSquareTest, ValidLinearMovementAtCornerAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a movement from the corner towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, 0, *this->configuration_space);  

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating an invalid movement with a linear movement model with a point inside but not on the configuration space edge in a square
TEST_F(MovementModelInSquareTest, InvalidInteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin at the center of the configuration space
    BURST::geometry::Point2D origin{5, 5};
    // Generate a movement from the interior towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point outside the configuration space in a square
TEST_F(MovementModelInSquareTest, InvalidExteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin outside the configuration space
    BURST::geometry::Point2D origin{67, 67};
    // Generate a movement from the exterior towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point on the edge but a direction pointing outside the configuration space in a square
TEST_F(MovementModelInSquareTest, InvalidLinearMovementPointingOutwardInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration space as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, -CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating a valid movement with a linear movement model in a concave configuration space
TEST_F(MovementModelInConcaveTest, ValidLinearMovementInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating an valid movement with a linear movement model in a concave configuration space at a concave vertex
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating a valid movement with a linear movement model in a concave configuration space along the edge
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAlongEdgeInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the right along the edge at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the movement to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid movement to have an endpoint, but got nullopt";
    // Expect the endpoint to not be the same as the origin, fail otherwise
    if (maybe_endpoint.has_value()) {
        EXPECT_NE(*maybe_endpoint, origin) << "Expected valid movement along the edge to have a different endpoint than the origin, but got the same point";
    } else {
        FAIL() << "Expected valid movement to have an endpoint, but got nullopt";
    }
}

// Test generating an invalid movement with a linear movement model in a concave configuration space at a concave vertex with a direction pointing outside the configuration space
TEST_F(MovementModelInConcaveTest, InvalidLinearMovementPointingOutwardAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration space as the origin
    BURST::geometry::Point2D origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the exterior at a -90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, -CGAL_PI/2, *this->configuration_space);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}


// -- MOVEMENTMODEL TRAJECTORY TESTS -------------------------------------------

// Test generating a valid trajectory with a linear movement model in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the bottom edge towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model at a corner in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryAtCornerInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a trajectory from the corner towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_space);
    
    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model along the edge in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the bottom edge towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, 0, *this->configuration_space);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model at a corner and along the edge in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryAtCornerAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a trajectory from the corner towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, 0, *this->configuration_space);
    
    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an invalid trajectory pointing outside the ConfigurationSpace in a square
TEST_F(MovementModelInSquareTest, InvalidLinearTrajectoryPointingOutwardInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, -CGAL_PI/4, *this->configuration_space);

    // Expect the trajectory to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_path.has_value()) << "Expected invalid path to not have a trajectory, but got a valid trajectory";
}

// Test generating a valid trajectory with a linear movement model in a concave ConfigurationSpace
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the first edge towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, CGAL_PI/2, *this->configuration_space);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an valid trajectory with a linear movement model in a concave ConfigurationSpace at a concave vertex
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->concave_vertex;
    // Generate a trajectory from the concave vertex towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, CGAL_PI/2, *this->configuration_space);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model in a concave ConfigurationSpace along the edge
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryAlongEdgeInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the first edge towards the right along the edge at a 45 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_space);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an invalid trajectory pointing outside the ConfigurationSpace in a concave ConfigurationSpace
TEST_F(MovementModelInConcaveTest, InvalidLinearTrajectoryPointingOutwardInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the ConfigurationSpace as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a trajectory from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::geometry::Segment2D> maybe_path = movement_model.generatePath(origin, -CGAL_PI/4, *this->configuration_space);

    // Expect the trajectory to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_path.has_value()) << "Expected invalid path to not have a trajectory, but got a valid trajectory";
}
