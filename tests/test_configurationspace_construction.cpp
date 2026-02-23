#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>

#include "test_helpers.hpp"

// Utility includes for tests

// Test for intended non-degeneracy of the configuration geometry with a regular polygon
TEST(ConfigurationGeometryConstructionTest, NonDegenerateRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(10, 0),
        BURST::geometry::Point2D(10, 10),
        BURST::geometry::Point2D(0, 10)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(config_geometry, nullptr) << "Expected non-degenerate ConfigurationGeometry for a regular polygon, but got nullptr.";
}

// Test for intended non-degeneracy of the configuration geometry with a simple polygon
TEST(ConfigurationGeometryConstructionTest, NonDegenerateSimplePolygon) {
    // Construct an already known non-degenerate WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D(0, 20),
        BURST::geometry::Point2D(-20, -20),
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(20, -20)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(config_geometry, nullptr) << "Expected non-degenerate ConfigurationGeometry for a simple polygon, but got nullptr.";
}

// Test for intended degeneracy of the configuration geometry with a too-small WallSpace
// In this case, it's too small to even fit the robot
TEST(ConfigurationGeometryConstructionTest, DegenerateTooSmallWallSpace) {
    // Construct a tiny square WallSpace that's smaller than the robot's radius
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(0.5, 0),
        BURST::geometry::Point2D(0.5, 0.5),
        BURST::geometry::Point2D(0, 0.5)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the configuration geometry to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(config_geometry, nullptr) << "Expected degenerate ConfigurationGeometry for a too-small WallSpace, but got a valid geometry.";
}

// Test for intended degeneracy of the configuration geometry with a tight-fitting WallSpace
// This should cause the configuration geometry to be degenerate since the translated edges will coincide and the intersection points will be collinear
TEST(ConfigurationGeometryConstructionTest, DegenerateTightFittingWallSpace) {
    // Construct a tight-fitting rectangular WallSpace that's exactly the height of the robot's diameter
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(10, 0),
        BURST::geometry::Point2D(10, 2),
        BURST::geometry::Point2D(0, 2)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the configuration geometry to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(config_geometry, nullptr) << "Expected degenerate ConfigurationGeometry for a tight-fitting WallSpace, but got a valid geometry.";
}
