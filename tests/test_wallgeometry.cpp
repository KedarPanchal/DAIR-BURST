#include <gtest/gtest.h>
#include <BURST/wall_geometry.hpp>

// Utility includes for tests
#include <optional>
#include <array>

// Test for intended non-degeneracy with a regular polygon
TEST(WallGeometryTest, NonDegenerateRegularPolygon) {
    // Construct a WallGeometry for a square
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0),
        Point_2(1, 0),
        Point_2(1, 1),
        Point_2(0, 1)
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_geometry.has_value());
}

// Test for intended non-degeneracy with a simple polygon
TEST(WallGeometryTest, NonDegenerateSimplePolygon) {
    // Construct a WallGeometry for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 2),
        Point_2(-2, -2),
        Point_2(0, 0),
        Point_2(2, -2)
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_geometry.has_value());
}

// Test for intended degeneracy with a straight line
TEST(WallGeometryTest, DegenerateStraightLine) {
    // Construct a WallGeometry for a straight line
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0),
        Point_2(1, 0),
        Point_2(2, 0)
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value());
}

// Test for intended degeneracy with a single point
TEST(WallGeometryTest, DegenerateSinglePoint) {
    // Construct a WallGeometry for a single point
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0)
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value());
}

// Test for intended degeneracy with a repeated point
TEST(WallGeometryTest, DegenerateRepeatedPoint) {
    // Construct a WallGeometry for a repeated point
    // This is a degenerate polygon
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(0, 0),
        Point_2(1, 0),
        Point_2(1, 0),
        Point_2(0, 0)
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value());
}

// Test for intended degeneracy with a self-intersecting polygon
// While this isn't truly degenerate, it is still invalid for our purposes and should be rejected
TEST(WallGeometryTest, DegenerateSelfIntersectingPolygon) {
    // Construct a WallGeometry for a self-intersecting polygon
    // This polygon looks like an hourglass and intersects itself at the center
    // This is a degenerate polygon for our purposes
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        Point_2(-2, 2),
        Point_2(2, 2),
        Point_2(-2, -2),
        Point_2(2, -2)
    });

    // Expect the WallGeometry to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_geometry.has_value());
}
