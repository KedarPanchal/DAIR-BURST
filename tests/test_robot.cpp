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
    BURST::geometry::WallSpace wall_space;

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
        // Construct the wall space with the hole
        std::optional<BURST::geometry::WallSpace> ws = BURST::geometry::WallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{10, 0},
            BURST::geometry::Point2D{10, 10},
            BURST::geometry::Point2D{0, 10}
        },
        {
            *hole_polygon
        });
        ASSERT_TRUE(ws.has_value()) << "Failed to construct wall space";
        this->wall_space = *ws;
    }
};

// -- ROBOT CONSTRUCTION TESTS -------------------------------------------------

// Test that a robot can be constructed with valid parameters with no explicit models or rotation seed
TEST(RobotTest, ValidConstruction) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{1, 1}, 0.1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
}

// Test that a robot can be constructed with valid parameters and a seed but no explicit models
TEST(RobotTest, ValidConstructionWithSeed) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(
        1.0, 
        BURST::geometry::Point2D{1, 1}, 
        0.1, 
        42
    );
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters and seed";
}

// Test that a robot can be constructed with valid parameters and explicit models
TEST(RobotTest, ValidConstructionWithModels) {
    auto robot = BURST::Robot<BURST::geometry::Ray2D, BURST::geometry::Segment2D, std::mt19937, BURST::numeric::flat_distribution>::create(
        1.0, 
        BURST::geometry::Point2D{1, 1}, 
        BURST::models::MaximumRotationModel{1}, 
        BURST::models::LinearMovementModel{}
    );
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters and explicit models";
}

// Test that a robot cannot be constructed with invalid parameters with no explicit models or rotation seed
TEST(RobotTest, InvalidConstruction) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(-1.0, BURST::geometry::Point2D{1, 1}, 0.1);
    ASSERT_FALSE(robot.has_value()) << "Successfully constructed robot with invalid parameters";
}

// Test that a robot cannot be constructed with invalid parameters and a seed but no explicit models
TEST(RobotTest, InvalidConstructionWithSeed) {
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(
        -1.0, 
        BURST::geometry::Point2D{1, 1}, 
        0.1, 
        42
    );
    ASSERT_FALSE(robot.has_value()) << "Successfully constructed robot with invalid parameters and seed";
}

// Test that a robot cannot be constructed with invalid parameters and explicit models
TEST(RobotTest, InvalidConstructionWithModels) {
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
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{6, 2}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of
    std::optional<std::monostate> result = this->wall_space.generateConfigurationSpace(*robot);
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
    std::optional<BURST::Robot<>> robot = BURST::Robot<>::create(1.0, BURST::geometry::Point2D{8, 6}, 1);
    ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters";
    // Assign it a configuration space that it is already on the boundary of the hole
    std::optional<std::monostate> result = this->wall_space.generateConfigurationSpace(*robot);
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
    std::optional<std::monostate> result = this->wall_space.generateConfigurationSpace(*robot);
    ASSERT_TRUE(result.has_value()) << "Failed to generate configuration space for robot";

    // Check that a warning was issued
    std::string warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(warning, "") << "Expected warning when setting configuration space that the robot is not on the boundary of, but got none";
}
