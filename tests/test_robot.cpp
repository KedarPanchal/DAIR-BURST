#include "BURST/numeric_types.hpp"
#include <gtest/gtest.h>

#include <BURST/robot.hpp>
#include <BURST/wall_space.hpp>
#include <BURST/geometric_types.hpp>
#include <BURST/models.hpp>

#include <optional>
#include <variant>

// -- TEST FIXTURE SETUP -------------------------------------------------------
class RobotTest : public ::testing::Test {
protected:
    std::optional<BURST::geometry::WallSpace> wall_space;

    void SetUp() override {
        // Construct a WallSpace for a square with a hole in the middle
        // Construct the hole polygon
        std::optional<BURST::geometry::Polygon2D> hole_polygon = BURST::geometry::construct_polygon({
            BURST::geometry::Point2D{4, 4},
            BURST::geometry::Point2D{6, 4},
            BURST::geometry::Point2D{6, 6},
            BURST::geometry::Point2D{4, 6}
        });
        ASSERT_TRUE(hole_polygon.has_value()) << "Failed to construct hole polygon";
        // Construct the wall space with the hol
        this->wall_space = BURST::geometry::WallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{10, 0},
            BURST::geometry::Point2D{10, 10},
            BURST::geometry::Point2D{0, 10}
        },
        {
            *hole_polygon
        });
        ASSERT_TRUE(wall_space.has_value()) << "Failed to construct wall space";
    }
};

// -- ROBOT CONSTRUCTION TESTS -------------------------------------------------

// Test that a robot can be constructed with valid parameters with no explicit models or rotation seed
TEST_F(RobotTest, ValidConstruction) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{1, 1}, 0.1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
}

// Test that a robot can be constructed with valid parameters and a seed but no explicit models
TEST_F(RobotTest, ValidConstructionWithSeed) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(
        1.0, 
        BURST::geometry::Point2D{1, 1}, 
        0.1, 
        42
    );
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters and seed";
}

// Test that a robot can be constructed with valid parameters and explicit models
TEST_F(RobotTest, ValidConstructionWithModels) {
    auto robot = BURST::Robot<BURST::geometry::Ray2D, BURST::geometry::Segment2D, std::mt19937, BURST::numeric::flat_distribution>::create(
        1.0, 
        BURST::geometry::Point2D{1, 1}, 
        BURST::models::MaximumRotationModel{1}, 
        BURST::models::LinearMovementModel{}
    );
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters and explicit models";
}

// Test that a robot cannot be constructed with invalid parameters with no explicit models or rotation seed
TEST_F(RobotTest, InvalidConstruction) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(-1.0, BURST::geometry::Point2D{1, 1}, 0.1);
    ASSERT_FALSE(robot.has_value()) << "Successfully constructed robot with invalid parameters";
}

// Test that a robot cannot be constructed with invalid parameters and a seed but no explicit models
TEST_F(RobotTest, InvalidConstructionWithSeed) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(
        -1.0, 
        BURST::geometry::Point2D{1, 1}, 
        0.1, 
        42
    );
    ASSERT_FALSE(robot.has_value()) << "Successfully constructed robot with invalid parameters and seed";
}

// Test that a robot cannot be constructed with invalid parameters and explicit models
TEST_F(RobotTest, InvalidConstructionWithModels) {
    auto robot = BURST::Robot<BURST::geometry::Ray2D, BURST::geometry::Segment2D, std::mt19937, BURST::numeric::flat_distribution>::create(
        -1.0, 
        BURST::geometry::Point2D{1, 1}, 
        BURST::models::MaximumRotationModel{1}, 
        BURST::models::LinearMovementModel{}
    );
    ASSERT_FALSE(robot.has_value()) << "Successfully constructed robot with invalid parameters and explicit models";
}


// -- ROBOT POSITION WARNING TESTS ---------------------------------------------

// Test that no warning is issued if the robot is given a ConfigurationSpace it's already on the boundary of
TEST_F(RobotTest, NoWarningOnConfigurationSpace) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{6, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting configuration space that the robot is already on the boundary of, but got: " << warning;
}

// Test that no warning is issued if the robot is given a ConfigurationSpace it's already on the hole boundary of
TEST_F(RobotTest, NoWarningOnConfigurationSpaceHoleBoundary) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{4, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting configuration space that the robot is already on the boundary of the hole, but got: " << warning;
}

// Test that a warning is issued if the robot is given a ConfigurationSpace it's not on the boundary of or hole boundary of
TEST_F(RobotTest, WarningOnConfigurationSpace) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{5, 5}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is not on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Check that a warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(warning, "") << "Expected warning when setting configuration space that the robot is not on the boundary of, but got none";
}

// Test that no warning is issued if the robot is given a new position that is on the boundary of the configuration space
TEST_F(RobotTest, NoWarningOnPosition) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{6, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";
    // Set a new position that is on the boundary of the configuration space
    robot->setPosition(BURST::geometry::Point2D{4, 1});

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting position that is on the boundary of the configuration space, but got: " << warning;
}

// Test that no warning is issued if the robot is given a new position that is on the boundary of the hole in the configuration space
TEST_F(RobotTest, NoWarningOnPositionHoleBoundary) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{4, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";
    // Set a new position that is on the boundary of the hole in the configuration space
    robot->setPosition(BURST::geometry::Point2D{5, 3});

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting position that is on the boundary of the hole in the configuration space, but got: " << warning;
}

// Test that no warning is issued if the robot starts on the configuration space boundary and is given a new position that is on the boundary of the hole in the configuration space
TEST_F(RobotTest, NoWarningOnPositionFromBoundaryToHoleBoundary) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{6, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";
    // Set a new position that is on the boundary of the hole in the configuration space
    robot->setPosition(BURST::geometry::Point2D{5, 3});

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting position from boundary of configuration space to boundary of hole in configuration space, but got: " << warning;
}

// Test that no warning is issued if the robot starts on the hole boundary and is given a new position that is on the boundary of the configuration space
TEST_F(RobotTest, NoWarningOnPositionFromHoleBoundaryToBoundary) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{4, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";
    // Set a new position that is on the boundary of the configuration space
    robot->setPosition(BURST::geometry::Point2D{4, 1});

    // Check that no warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_EQ(warning, "") << "Expected no warning when setting position from boundary of hole in configuration space to boundary of configuration space, but got: " << warning;
}

// Test that a warning is issued if the robot is given a new position that is not on the boundary of the configuration space or hole boundary
TEST_F(RobotTest, WarningOnPosition) {
    // Capture stderr to check for warnings
    testing::internal::CaptureStderr();

    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{6, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";
    // Set a new position that is not on the boundary of the configuration space or hole boundary
    robot->setPosition(BURST::geometry::Point2D{5, 5});

    // Check that a warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(warning, "") << "Expected warning when setting position that is not on the boundary of the configuration space or hole boundary, but got none";
}


// -- ROBOT RAYCAST TESTS ------------------------------------------------------

// Test that a raycast from a robot on the boundary of the configuration space to another spot on the boundary of the configuration space is valid
TEST_F(RobotTest, RaycastFromBoundaryToBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Perform a raycast from the robot's position to another spot on the boundary of the configuration space
    std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(3 * CGAL_PI/4);
    // Expect the raycast to be valid
    // 
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid raycast from boundary to boundary, but got nullopt";
}

// Test that a raycast from a robot on the boundary of the configuration space to a spot on the hole boundary of the configuration space is valid
TEST_F(RobotTest, RaycastFromBoundaryToHoleBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Perform a raycast from the robot's position to a spot on the hole boundary of the configuration space
    std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(CGAL_PI/4);
    // Expect the raycast to be valid
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid raycast from boundary to hole boundary, but got nullopt";
}

// Test that a raycast from a robot on the hole boundary of the configuration space to a spot on the boundary of the configuration space is valid
TEST_F(RobotTest, RaycastFromHoleBoundaryToBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{5, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Perform a raycast from the robot's position to a spot on the boundary of the configuration space
    std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(5 * CGAL_PI/4);
    // Expect the raycast to be valid
    EXPECT_TRUE(maybe_endpoint.has_value()) << "Expected valid raycast from hole boundary to boundary, but got nullopt";
}

// Test that a raycast from a robot on the boundary of the configuration space to a spot outside the configuration space is invalid
TEST_F(RobotTest, RaycastFromBoundaryToExterior) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Perform a raycast from the robot's position to a spot outside the configuration space
    std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(-CGAL_PI/2);
    // Expect the raycast to be invalid
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid raycast from boundary to exterior to not have an endpoint, but got a valid endpoint at (" << maybe_endpoint->x() << ", " << maybe_endpoint->y() << ")";
}

// Test that a raycast from a robot on the hole boundary of the configuration space to the interior of the hole is invalid
TEST_F(RobotTest, RaycastFromHoleBoundaryToHoleInterior) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{5, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Perform a raycast from the robot's position to the interior of the hole
    std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(CGAL_PI/2);
    // Expect the raycast to be invalid
    EXPECT_FALSE(maybe_endpoint.has_value()) << "Expected invalid raycast from hole boundary to hole interior to not have an endpoint, but got a valid endpoint at (" << maybe_endpoint->x() << ", " << maybe_endpoint->y() << ")";
}

// -- ROBOT STADIUM GENERATION TESTS -------------------------------------------

// Test generating a stadium for a motion from the boundary of the configuration space to another spot on the boundary of the configuration space
TEST_F(RobotTest, StadiumFromBoundaryToBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Generate a stadium for a motion from the robot's position to another spot on the boundary of the configuration space
    std::optional<BURST::geometry::CurvilinearPolygonSet2D> maybe_stadium = robot->coveredArea(3 * CGAL_PI/4);
    // Expect the stadium to be valid
    EXPECT_TRUE(maybe_stadium.has_value()) << "Expected valid stadium from boundary to boundary, but got nullopt";
    
    if (maybe_stadium.has_value()) {
        // Check that the stadium contains the start, end, and midpoint of the robot's trajectory 
        std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(3 * CGAL_PI/4);
        ASSERT_TRUE(maybe_endpoint.has_value()) << "Failed to find endpoint of robot's trajectory";
        BURST::geometry::Point2D midpoint = BURST::geometry::midpoint(robot->getPosition(), *maybe_endpoint);

        BURST::CurvedTraits::Point_2 converted_start = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(robot->getPosition());
        BURST::CurvedTraits::Point_2 converted_end = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(*maybe_endpoint);
        BURST::CurvedTraits::Point_2 converted_midpoint = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(midpoint);

        EXPECT_TRUE(maybe_stadium->oriented_side(converted_start) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's starting position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_end) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's ending position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_midpoint) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain midpoint of robot's trajectory, but it does not";
    } else {
        FAIL() << "Expected valid stadium from boundary to boundary, but got nullopt";
    }
}

// Test generating a stadium for a motion from the boundary of the configuration space to a spot on the hole boundary of the configuration space
TEST_F(RobotTest, StadiumFromBoundaryToHoleBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Generate a stadium for a motion from the robot's position to a spot on the hole boundary of the configuration space
    std::optional<BURST::geometry::CurvilinearPolygonSet2D> maybe_stadium = robot->coveredArea(CGAL_PI/4);
    // Expect the stadium to be valid
    EXPECT_TRUE(maybe_stadium.has_value()) << "Expected valid stadium from boundary to hole boundary, but got nullopt";

    if (maybe_stadium.has_value()) {
        // Check that the stadium contains the start, end, and midpoint of the robot's trajectory 
        std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(CGAL_PI/4);
        ASSERT_TRUE(maybe_endpoint.has_value()) << "Failed to find endpoint of robot's trajectory";
        BURST::geometry::Point2D midpoint = BURST::geometry::midpoint(robot->getPosition(), *maybe_endpoint);

        BURST::CurvedTraits::Point_2 converted_start = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(robot->getPosition());
        BURST::CurvedTraits::Point_2 converted_end = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(*maybe_endpoint);
        BURST::CurvedTraits::Point_2 converted_midpoint = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(midpoint);

        EXPECT_TRUE(maybe_stadium->oriented_side(converted_start) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's starting position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_end) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's ending position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_midpoint) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain midpoint of robot's trajectory, but it does not";
    } else {
        FAIL() << "Expected valid stadium from boundary to hole boundary, but got nullopt";
    }
}

// Test generating a stadium for a motion from the hole boundary of the configuration space to a spot on the boundary of the configuration space
TEST_F(RobotTest, StadiumFromHoleBoundaryToBoundary) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{5, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Generate a stadium for a motion from the robot's position to a spot on the boundary of the configuration space
    std::optional<BURST::geometry::CurvilinearPolygonSet2D> maybe_stadium = robot->coveredArea(5 * CGAL_PI/4);
    // Expect the stadium to be valid
    EXPECT_TRUE(maybe_stadium.has_value()) << "Expected valid stadium from hole boundary to boundary, but got nullopt";

    if (maybe_stadium.has_value()) {
        // Check that the stadium contains the start, end, and midpoint of the robot's trajectory 
        std::optional<BURST::geometry::Point2D> maybe_endpoint = robot->shootRay(5 * CGAL_PI/4);
        ASSERT_TRUE(maybe_endpoint.has_value()) << "Failed to find endpoint of robot's trajectory";
        BURST::geometry::Point2D midpoint = BURST::geometry::midpoint(robot->getPosition(), *maybe_endpoint);

        BURST::CurvedTraits::Point_2 converted_start = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(robot->getPosition());
        BURST::CurvedTraits::Point_2 converted_end = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(*maybe_endpoint);
        BURST::CurvedTraits::Point_2 converted_midpoint = BURST::geometry::convert_point<BURST::CurvedTraits::Point_2>(midpoint);
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_start) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's starting position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_end) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain robot's ending position, but it does not";
        EXPECT_TRUE(maybe_stadium->oriented_side(converted_midpoint) == CGAL::ON_POSITIVE_SIDE) << "Expected stadium to contain midpoint of robot's trajectory, but it does not";
    } else {
        FAIL() << "Expected valid stadium from hole boundary to boundary, but got nullopt";
    }
}

// Test generating a stadium for a motion from the boundary of the configuration space to a spot outside the configuration space
TEST_F(RobotTest, StadiumFromBoundaryToExterior) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{3, 1}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Generate a stadium for a motion from the robot's position to a spot outside the configuration space
    std::optional<BURST::geometry::CurvilinearPolygonSet2D> maybe_stadium = robot->coveredArea(-CGAL_PI/2);
    // Expect the stadium to be invalid
    EXPECT_FALSE(maybe_stadium.has_value()) << "Expected invalid stadium from boundary to exterior to be nullopt, but got a valid stadium";
}

// Test generating a stadium for a motion from the hole boundary of the configuration space to the interior of the hole
TEST_F(RobotTest, StadiumFromHoleBoundaryToHoleInterior) {
    // Construct the robot
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{5, 3}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space->generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Generate a stadium for a motion from the robot's position to the interior of the hole
    std::optional<BURST::geometry::CurvilinearPolygonSet2D> maybe_stadium = robot->coveredArea(CGAL_PI/2);
    // Expect the stadium to be invalid
    EXPECT_FALSE(maybe_stadium.has_value()) << "Expected invalid stadium from hole boundary to hole interior to be nullopt, but got a valid stadium";
}
