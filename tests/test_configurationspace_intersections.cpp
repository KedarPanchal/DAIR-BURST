#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>

#include "test_helpers.hpp"

// Utility includes for tests
#include <optional>
#include <vector>
#include <iterator>

// Test point intersection for a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST(ConfigurationSpaceIntersectionTest, PointIntersectionRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    // Expect the WallSpace to be non-degenerate
    // i.e. it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e. it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";
    
    // Create a point that lies on the edge of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{1, 5};

    // Expect the point to intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is not nullopt
    EXPECT_TRUE(configuration_space->intersection(intersection_point).has_value()) << "Expected point to intersect with the ConfigurationSpace, but got nullopt";
}

// Test point intersection at a corner for a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST(ConfigurationSpaceIntersectionTest, PointIntersectionAtCornerRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    // Expect the WallSpace to be non-degenerate
    // i.e. it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";
    
    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e. it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";

    // Create a point that lies on the corner of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{1, 1};

    // Expect the point to intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is not nullopt
    EXPECT_TRUE(configuration_space->intersection(intersection_point).has_value()) << "Expected point to intersect with the ConfigurationSpace, but got nullopt";
}

// Test invalid point intersection for a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST(ConfigurationSpaceIntersectionTest, InvalidPointIntersectionRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    // Expect the WallSpace to be non-degenerate
    // i.e. it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e. it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";

    // Create a point that lies not on the boundary of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{5, 5};

    // Expect the point to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is nullopt
    EXPECT_FALSE(configuration_space->intersection(intersection_point).has_value()) << "Expected point to not intersect with the ConfigurationSpace, but got a valid intersection";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon
TEST(ConfigurationSpaceIntersectionTest, RayIntersectionRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    // Expect the WallSpace to be non-degenerate
    // i.e. it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e. it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";

    // Create a ray that starts on the edge of the ConfigurationSpace and points inward
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{1, 5}, BURST::geometry::Vector2D{1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Point2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}  
