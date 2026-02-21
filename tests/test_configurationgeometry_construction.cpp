#include <gtest/gtest.h>
#include <BURST/configuration_geometry.hpp>
#include <BURST/wall_geometry.hpp>

#include "test_helpers.hpp"

// Utility includes for tests

// Test for intended non-degeneracy of the configuration geometry with a regular polygon
TEST(ConfigurationGeometryConstructionTest, NonDegenerateRegularPolygon) {
    // Construct an already known non-degenerate WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point2D<BURST::RationalKernel>(0, 0),
        BURST::Point2D<BURST::RationalKernel>(10, 0),
        BURST::Point2D<BURST::RationalKernel>(10, 10),
        BURST::Point2D<BURST::RationalKernel>(0, 10)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(config_geometry, nullptr) << "Expected non-degenerate ConfigurationGeometry for a regular polygon, but got nullptr.";
}

// Test for intended non-degeneracy of the configuration geometry with a simple polygon
TEST(ConfigurationGeometryConstructionTest, NonDegenerateSimplePolygon) {
    // Construct an already known non-degenerate WallGeometry for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point2D<BURST::RationalKernel>(0, 20),
        BURST::Point2D<BURST::RationalKernel>(-20, -20),
        BURST::Point2D<BURST::RationalKernel>(0, 0),
        BURST::Point2D<BURST::RationalKernel>(20, -20)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(config_geometry, nullptr) << "Expected non-degenerate ConfigurationGeometry for a simple polygon, but got nullptr.";
}

// Test for intended degeneracy of the configuration geometry with a too-small WallGeometry
// In this case, it's too small to even fit the robot
TEST(ConfigurationGeometryConstructionTest, DegenerateTooSmallWallGeometry) {
    // Construct a tiny square WallGeometry that's smaller than the robot's radius
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point2D<BURST::RationalKernel>(0, 0),
        BURST::Point2D<BURST::RationalKernel>(0.5, 0),
        BURST::Point2D<BURST::RationalKernel>(0.5, 0.5),
        BURST::Point2D<BURST::RationalKernel>(0, 0.5)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(config_geometry, nullptr) << "Expected degenerate ConfigurationGeometry for a too-small WallGeometry, but got a valid geometry.";
}

// Test for intended degeneracy of the configuration geometry with a tight-fitting WallGeometry
// This should cause the configuration geometry to be degenerate since the translated edges will coincide and the intersection points will be collinear
TEST(ConfigurationGeometryConstructionTest, DegenerateTightFittingWallGeometry) {
    // Construct a tight-fitting rectangular WallGeometry that's exactly the height of the robot's diameter
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point2D<BURST::RationalKernel>(0, 0),
        BURST::Point2D<BURST::RationalKernel>(10, 0),
        BURST::Point2D<BURST::RationalKernel>(10, 2),
        BURST::Point2D<BURST::RationalKernel>(0, 2)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(config_geometry, nullptr) << "Expected degenerate ConfigurationGeometry for a tight-fitting WallGeometry, but got a valid geometry.";
}
