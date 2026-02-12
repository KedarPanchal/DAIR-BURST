#include <gtest/gtest.h>
#include <BURST/configuration_geometry.hpp>
#include <BURST/wall_geometry.hpp>
#include <BURST/robot.hpp>

// Utility includes for tests
#include <optional>

// Create a test fixture for a common robot setup
class ConfigurationGeometryConstructionTest : public ::testing::Test {
protected:
    fscalar robot_radius = 0.5;
    fscalar max_rotation_error = 0.1;
    BURST::Robot robot{robot_radius, max_rotation_error};
};

// Test for intended non-degeneracy of configuration geometry with a regular polygon
TEST_F(ConfigurationGeometryConstructionTest, NonDegenerateRegularPolygon) {
    // Construct a WallGeometry for a square
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0),
        Point_2(10, 0),
        Point_2(10, 10),
        Point_2(0, 10)
    });

    // Generate the configuration geometry for the robot
    auto config_geometry = wall_geometry->generateConfigurationGeometry(robot);

    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(config_geometry.has_value()) << "Expected non-degenerate configuration geometry for a regular polygon, but got nullopt.";
}

// Test for intended non-degeneracy of configuration geometry with a simple polygon
TEST_F(ConfigurationGeometryConstructionTest, NonDegenerateSimplePolygon) {
    // Construct a WallGeometry for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 2),
        Point_2(-20, -20),
        Point_2(0, 0),
        Point_2(20, -20)
    });

    // Generate the configuration geometry for the robot
    auto config_geometry = wall_geometry->generateConfigurationGeometry(robot);

    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(config_geometry.has_value()) << "Expected non-degenerate configuration geometry for a simple polygon, but got nullopt.";
}

// Test for intended degeneracy of configuration geometry with a wall geometry smaller than the robot's radius
TEST_F(ConfigurationGeometryConstructionTest, DegenerateSmallWallGeometry) {
    // Construct a WallGeometry for a small square
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0),
        Point_2(0.1, 0),
        Point_2(0.1, 0.1),
        Point_2(0, 0.1)
    });

    // Generate the configuration geometry for the robot
    auto config_geometry = wall_geometry->generateConfigurationGeometry(robot);

    // Expect the configuration geometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(config_geometry.has_value()) << "Expected degenerate configuration geometry for a wall geometry smaller than the robot's radius, but got a valid geometry.";
}
