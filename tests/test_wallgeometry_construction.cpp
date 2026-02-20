#include <gtest/gtest.h>
#include <BURST/wall_geometry.hpp>

// Utility includes for tests
#include <optional>

// Test for intended non-degeneracy with a regular polygon
TEST(WallGeometryConstructionTest, NonDegenerateRegularPolygon) {
    // Construct a WallGeometry for a square
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0},
        BURST::Point2D{10, 0},
        BURST::Point2D{10, 10},
        BURST::Point2D{0, 10}
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_geometry.has_value()) << "Expected non-degenerate WallGeometry for a regular polygon, but got nullopt.";
}

// Test for intended non-degeneracy with a simple polygon
TEST(WallGeometryConstructionTest, NonDegenerateSimplePolygon) {
    // Construct a WallGeometry for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 20},
        BURST::Point2D{-20, -20},
        BURST::Point2D{0, 0},
        BURST::Point2D{20, -20}
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_geometry.has_value()) << "Expected non-degenerate WallGeometry for a simple polygon, but got nullopt.";
}

// Test for intended degeneracy with a straight line
TEST(WallGeometryConstructionTest, DegenerateStraightLine) {
    // Construct a WallGeometry for a straight line
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0},
        BURST::Point2D{10, 0},
        BURST::Point2D{20, 0}
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value()) << "Expected degenerate WallGeometry for a straight line, but got a valid geometry.";
}

// Test for intended degeneracy with a single point
TEST(WallGeometryConstructionTest, DegenerateSinglePoint) {
    // Construct a WallGeometry for a single point
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0}
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value()) << "Expected degenerate WallGeometry for a single point, but got a valid geometry.";
}

// Test for intended degeneracy with a repeated point
TEST(WallGeometryConstructionTest, DegenerateRepeatedPoint) {
    // Construct a WallGeometry for a repeated point
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0},
        BURST::Point2D{10, 0},
        BURST::Point2D{10, 0},
        BURST::Point2D{0, 0}
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value()) << "Expected degenerate WallGeometry for a repeated point, but got a valid geometry.";
}

// Test for intended degeneracy with a self-intersecting polygon
// While this isn't truly degenerate, it is still invalid for our purposes and should be rejected
TEST(WallGeometryConstructionTest, DegenerateSelfIntersectingPolygon) {
    // Construct a WallGeometry for a self-intersecting polygon
    // This polygon looks like an hourglass and intersects itself at the center
    // This is a degenerate polygon for our purposes
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{-20, 20},
        BURST::Point2D{20, 20},
        BURST::Point2D{-20, -20},
        BURST::Point2D{20, -20}
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value()) << "Expected degenerate WallGeometry for a self-intersecting polygon, but got a valid geometry.";
}

// Test for intended degeneracy with a a normally valid polygon with out-of-order points
TEST(WallGeometryConstructionTest, DegenerateOutOfOrderPoints) {
    // Construct a WallGeometry for a normally valid polygon with out-of-order points
    // This polygon is a square but the points are given in an order that creates a bowtie shape
    // This is a degenerate polygon for our purposes
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0},
        BURST::Point2D{10, 10},
        BURST::Point2D{10, 0},
        BURST::Point2D{0, 10}
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value()) << "Expected degenerate WallGeometry for a normally valid polygon with out-of-order points, but got a valid geometry.";
}
