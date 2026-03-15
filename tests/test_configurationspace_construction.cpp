#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>

#include "test_helpers.hpp"

// -- NON-DEGENERATE NON-HOLED POLYGON TESTS -----------------------------------

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygon) {
    // Construct an already known non-degenerate WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(10, 0),
        BURST::geometry::Point2D(10, 10),
        BURST::geometry::Point2D(0, 10)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a simple polygon
TEST(ConfigurationSpaceConstructionTest, NonDegenerateSimplePolygon) {
    // Construct an already known non-degenerate WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D(0, 20),
        BURST::geometry::Point2D(-20, -20),
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(20, -20)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a simple polygon, but got nullptr.";
}


// -- NON-DEGENERATE HOLED POLYGON TESTS ---------------------------------------

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with a hole in the middle
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithHole) {
    // Construct an already known non-degenerate WallSpace for a square with a hole in the middle
    // Construct a square hole
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D(6, 4),
        BURST::geometry::Point2D(6, 6),
        BURST::geometry::Point2D(4, 6),
        BURST::geometry::Point2D(4, 4)
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the middle hole inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(10, 0),
        BURST::geometry::Point2D(10, 10),
        BURST::geometry::Point2D(0, 10)
    },
    {
        *hole
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";
    
    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with a hole in the middle, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with multiple holes in the middle
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithHoles) {
    // Construct an already known non-degenerate WallSpace for a square with multiple holes in the middle
    // Construct the first 2x2 square hole
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{5, 6},
        BURST::geometry::Point2D{5, 8},
        BURST::geometry::Point2D{3, 8},
        BURST::geometry::Point2D{3, 6}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate first hole.";

    // Construct the second 2x1 hole
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{7, 2},
        BURST::geometry::Point2D{7, 3},
        BURST::geometry::Point2D{5, 3},
        BURST::geometry::Point2D{5, 2}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate second hole.";

    // Construct a WallSpace for a square with the middle holes inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole1,
        *hole2
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with multiple holes in the middle, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with a hole in the middle that is close to the outer boundary
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithHoleCloseToBoundary) {
    // Construct an already known non-degenerate WallSpace for a square with a hole in the middle that's close to the outer boundary
    // Construct a square hole close to the boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{3, 1},
        BURST::geometry::Point2D{3, 3},
        BURST::geometry::Point2D{1, 3},
        BURST::geometry::Point2D{1, 1}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the hole close to the boundary
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {        
        *hole
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with a hole in the middle that's close to the outer boundary, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with a multiple holes in the middle that result in an overlapping configuration space
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithOverlappingHoles) {
    // Construct an already known non-degenerate WallSpace for a square with multiple holes in the middle that result in an overlapping configuration space
    // Construct the first 2x2 square hole
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{5, 6},
        BURST::geometry::Point2D{5, 8},
        BURST::geometry::Point2D{3, 8},
        BURST::geometry::Point2D{3, 6}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate first hole.";

    // Construct the second 2x1 hole that overlaps with the first hole's configuration space
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{8, 5},
        BURST::geometry::Point2D{8, 6},
        BURST::geometry::Point2D{6, 6},
        BURST::geometry::Point2D{6, 5}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate second hole.";

    // Construct a WallSpace for a square with the middle holes inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole1,
        *hole2
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with multiple holes in the middle that result in an overlapping configuration space, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with a simply polygon hole in the middle
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithSimpleHole) {
    // Construct an already known non-degenerate WallSpace for a square with a simple polygon hole in the middle
    // Construct the simple polygon hole with an arrowhead shape
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{6, 5},
        BURST::geometry::Point2D{5, 7},
        BURST::geometry::Point2D{4, 5},
        BURST::geometry::Point2D{5, 6}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the middle hole inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with a simple hole in the middle, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polygon with a simple polygon hole in the middle that is close to the outer boundary
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithSimpleHoleCloseToBoundary) {
    // Construct an already known non-degenerate WallSpace for a square with a simple polygon hole in the middle
    // Construct the simple polygon hole with an arrowhead shape
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{9, 7},
        BURST::geometry::Point2D{8, 9},
        BURST::geometry::Point2D{7, 7},
        BURST::geometry::Point2D{8, 8}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the middle hole inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with a simple hole in the middle, but got nullptr.";
}

// Test for intended non-degeneracy of the ConfigurationSpace with a regular polywgon with multiple simple polygon holes in the middle that result in an overlapping configuration space
TEST(ConfigurationSpaceConstructionTest, NonDegenerateRegularPolygonWithOverlappingSimpleHoles) {
    // Construct an already known non-degenerate WallSpace for a square with a simple polygon hole in the middle
    // Construct the first simple polygon hole with an arrowhead shape
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{6, 5},
        BURST::geometry::Point2D{5, 7},
        BURST::geometry::Point2D{4, 5},
        BURST::geometry::Point2D{5, 6}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate first hole.";

    // Construct the second simple polygon hole with an arrowhead shape that overlaps with the first hole's configuration space
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{4, 6},
        BURST::geometry::Point2D{4, 8},
        BURST::geometry::Point2D{3, 7},
        BURST::geometry::Point2D{2, 7}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate second hole.";

    // Construct a WallSpace for a square with the middle hole inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole1,
        *hole2
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    EXPECT_NE(configuration_space, nullptr) << "Expected non-degenerate ConfigurationSpace for a regular polygon with a simple hole in the middle, but got nullptr.";
}


// -- DEGENERATE NON-HOLED POLYGON TESTS ---------------------------------------

// Test for intended degeneracy of the ConfigurationSpace with a too-small WallSpace
// In this case, it's too small to even fit the robot
TEST(ConfigurationSpaceConstructionTest, DegenerateTooSmallWallSpace) {
    // Construct a tiny square WallSpace that's smaller than the robot's radius
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(0.5, 0),
        BURST::geometry::Point2D(0.5, 0.5),
        BURST::geometry::Point2D(0, 0.5)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(configuration_space, nullptr) << "Expected degenerate ConfigurationSpace for a too-small WallSpace, but got a valid geometry.";
}

// Test for intended degeneracy of the ConfigurationSpace with a tight-fitting WallSpace
// This should cause the ConfigurationSpace to be degenerate since the translated edges will coincide and the intersection points will be collinear
TEST(ConfigurationSpaceConstructionTest, DegenerateTightFittingWallSpace) {
    // Construct a tight-fitting rectangular WallSpace that's exactly the height of the robot's diameter
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D(0, 0),
        BURST::geometry::Point2D(10, 0),
        BURST::geometry::Point2D(10, 2),
        BURST::geometry::Point2D(0, 2)
    });
    // For some reason if the WallSpace is degenerate, crash the test
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be degenerate
    // i.e., it is nullptr
    EXPECT_EQ(configuration_space, nullptr) << "Expected degenerate ConfigurationSpace for a tight-fitting WallSpace, but got a valid geometry.";
}
