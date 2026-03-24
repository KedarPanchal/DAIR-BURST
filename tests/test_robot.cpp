#include "BURST/numeric_types.hpp"
#include <gtest/gtest.h>

#include <BURST/robot.hpp>
#include <BURST/wall_space.hpp>
#include <BURST/geometric_types.hpp>
#include <BURST/models.hpp>

// -- TEST FIXTURE SETUP -------------------------------------------------------
class RobotTest : public ::testing::Test {
    BURST::geometry::WallSpace wall_space;
protected:
    void SetUp() override {
        // Construct a WallSpace for a square with a hole in the middle
        // Construct the hole polygon
        std::optional<BURST::geometry::Polygon2D> hole_polygon = BURST::geometry::construct_polygon({
            BURST::geometry::Point2D{6, 6},
            BURST::geometry::Point2D{14, 6},
            BURST::geometry::Point2D{14, 14},
            BURST::geometry::Point2D{6, 14}
        });
        ASSERT_TRUE(hole_polygon.has_value()) << "Failed to construct hole polygon";
        // Construct the wall space with the hole
        std::optional<BURST::geometry::WallSpace> ws = BURST::geometry::WallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{20, 0},
            BURST::geometry::Point2D{20, 20},
            BURST::geometry::Point2D{0, 20}
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
