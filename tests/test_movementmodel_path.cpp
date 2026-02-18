#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests
#include "test_helpers.hpp"
#include <optional>

// Test generating a valid trajectory with a linear movement model in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the bottom edge towards the interior at a 45 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a lienar movement model at a corner in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryAtCornerInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the corner vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->corner_vertex;
    // Generate a trajectory from the corner towards the interior at a 45 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_geometry);
    
    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model along the edge in a square
TEST_F(MovementModelInSquareTest, ValidLinearTrajectoryAlongEdgeInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the bottom edge towards the right along the edge at a 0 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, 0, *this->configuration_geometry);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an invalid trajectory pointing outside the configuration geometry in a square
TEST_F(MovementModelInSquareTest, InvalidLinearTrajectoryPointingOutwardInSquare) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the bottom edge of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, -CGAL_PI/4, *this->configuration_geometry);

    // Expect the trajectory to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_path.has_value()) << "Expected invalid path to not have a trajectory, but got a valid trajectory";
}

// Test generating a valid trajectory with a linear movement model in a concave configuration geometry
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the first edge towards the interior at a 90 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, CGAL_PI/2, *this->configuration_geometry);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an valid trajectory with a linear movement model in a concave configuration geometry at a concave vertex
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryAtConcaveCornerInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->concave_vertex;
    // Generate a trajectory from the concave vertex towards the interior at a 90 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, CGAL_PI/2, *this->configuration_geometry);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating a valid trajectory with a linear movement model in a concave configuration geometry along the edge
TEST_F(MovementModelInConcaveTest, ValidLinearTrajectoryAlongEdgeInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the first edge towards the right along the edge at a 45 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, CGAL_PI/4, *this->configuration_geometry);

    // Expect the trajectory to be valid
    // i.e., it is not nullopt
    EXPECT_TRUE(maybe_path.has_value()) << "Expected valid path, but got nullopt";
}

// Test generating an invalid trajectory pointing outside the configuration geometry in a concave configuration geometry
TEST_F(MovementModelInConcaveTest, InvalidLinearTrajectoryPointingOutwardInConcave) {
    // Construct a LinearMovementModel
    auto movement_model = BURST::models::LinearMovementModel{};
    // Use the midpoint of the edge containing the concave vertex of the configuration geometry as the origin
    BURST::Point_2 origin = this->edge_midpoint;
    // Generate a trajectory from the edge towards the exterior at a 45 degree angle
    std::optional<BURST::Segment_2> maybe_path = movement_model.generatePath(origin, -CGAL_PI/4, *this->configuration_geometry);

    // Expect the trajectory to be invalid
    // i.e., it is nullopt
    EXPECT_FALSE(maybe_path.has_value()) << "Expected invalid path to not have a trajectory, but got a valid trajectory";
}
