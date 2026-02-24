#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests
#include "test_helpers.hpp"
#include <optional>

// Test generating a valid movement with a linear movement model in a square
// This means that the movement should be in a straight line towards the interior of the configuration geometry
TEST_F(MovementModelInSquareTest, ValidLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

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
    // Use the corner vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a movement from the corner towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

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
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the bottom edge towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, 0, *this->configuration_geometry);

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
    // Use the corner vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->corner_vertex;
    // Generate a movement from the corner towards the right along the edge at a 0 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, 0, *this->configuration_geometry);  

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

// Test generating an invalid movement with a linear movement model with a point inside but not on the configuration geometry edge in a square
TEST_F(MovementModelInSquareTest, InvalidInteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin at the center of the configuration geometry
    BURST::geometry::Point2D origin{5, 5};
    // Generate a movement from the interior towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point outside the configuration geometry in a square
TEST_F(MovementModelInSquareTest, InvalidExteriorLinearMovementInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Specify an origin outside the configuration geometry
    BURST::geometry::Point2D origin{67, 67};
    // Generate a movement from the exterior towards the interior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating an invalid movement with a linear movement model with a point on the edge but a direction pointing outside the configuration geometry in a square
TEST_F(MovementModelInSquareTest, InvalidLinearMovementPointingOutwardInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, -CGAL_PI/4, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}

// Test generating a valid movement with a linear movement model in a concave configuration geometry
TEST_F(MovementModelInConcaveTest, ValidLinearMovementInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_geometry);

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

// Test generating an valid movement with a linear movement model in a concave configuration geometry at a concave vertex
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the interior at a 90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/2, *this->configuration_geometry);

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

// Test generating a valid movement with a linear movement model in a concave configuration geometry along the edge
TEST_F(MovementModelInConcaveTest, ValidLinearMovementAlongEdgeInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->edge_midpoint;
    // Generate a movement from the first edge towards the right along the edge at a 45 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, CGAL_PI/4, *this->configuration_geometry);

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

// Test generating an invalid movement with a linear movement model in a concave configuration geometry at a concave vertex with a direction pointing outside the configuration geometry
TEST_F(MovementModelInConcaveTest, InvalidLinearMovementPointingOutwardAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration geometry as the origin
    BURST::geometry::Point2D origin = this->concave_vertex;
    // Generate a movement from the concave vertex towards the exterior at a -90 degree angle
    std::optional<BURST::geometry::Point2D> maybe_endpoint = movement_model(origin, -CGAL_PI/2, *this->configuration_geometry);

    // Expect the movement to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid movement to not have an endpoint, but got a valid endpoint";
}
