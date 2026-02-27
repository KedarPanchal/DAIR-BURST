#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/wall_space.hpp>

// Utility includes for tests
#include <optional>

// Test for intended non-degeneracy with a regular polygon
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygon) {
    // Construct a WallSpace for a square
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon, but got nullopt.";
}

// Test for intended non-degeneracy with a simple polygon
TEST(WallSpaceConstructionTest, NonDegenerateSimplePolygon) {
    // Construct a WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a simple polygon, but got nullopt.";
}

// Test for intended degeneracy with a straight line
TEST(WallSpaceConstructionTest, DegenerateStraightLine) {
    // Construct a WallSpace for a straight line
    // This is a degenerate polygon
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{20, 0}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a straight line, but got a valid geometry.";
}

// Test for intended degeneracy with a single point
TEST(WallSpaceConstructionTest, DegenerateSinglePoint) {
    // Construct a WallSpace for a single point
    // This is a degenerate polygon
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a single point, but got a valid geometry.";
}

// Test for intended degeneracy with a repeated point
TEST(WallSpaceConstructionTest, DegenerateRepeatedPoint) {
    // Construct a WallSpace for a repeated point
    // This is a degenerate polygon
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{0, 0}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a repeated point, but got a valid geometry.";
}

// Test for intended degeneracy with a self-intersecting polygon
// While this isn't truly degenerate, it is still invalid for our purposes and should be rejected
TEST(WallSpaceConstructionTest, DegenerateSelfIntersectingPolygon) {
    // Construct a WallSpace for a self-intersecting polygon
    // This polygon looks like an hourglass and intersects itself at the center
    // This is a degenerate polygon for our purposes
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{-20, 20},
        BURST::geometry::Point2D{20, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a self-intersecting polygon, but got a valid geometry.";
}

// Test for intended degeneracy with a a normally valid polygon with out-of-order points
TEST(WallSpaceConstructionTest, DegenerateOutOfOrderPoints) {
    // Construct a WallSpace for a normally valid polygon with out-of-order points
    // This polygon is a square but the points are given in an order that creates a bowtie shape
    // This is a degenerate polygon for our purposes
    auto wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a normally valid polygon with out-of-order points, but got a valid geometry.";
}
