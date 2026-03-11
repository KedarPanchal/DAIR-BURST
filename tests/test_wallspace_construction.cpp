#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/wall_space.hpp>

// Utility includes for tests
#include <optional>


// -- NON-DEGENERATE NON-HOLED POLYGON TESTS -----------------------------------

// Test for intended non-degeneracy with a regular polygon
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygon) {
    // Construct a WallSpace for a square
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a simple polygon, but got nullopt.";
}


// -- NON-DEGENERATE HOLED POLYGON TESTS ---------------------------------------

// Test for intended non-degeneracy with a regular polygon with a single regular hole in the middle
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygonWithHole) {
    // Construct a square hole
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{6, 4},
        BURST::geometry::Point2D{6, 6},
        BURST::geometry::Point2D{4, 6},
        BURST::geometry::Point2D{4, 4}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the middle hole inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    }, 
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon with a hole in the middle, but got nullopt.";
}

// Test for intended non-degeneracy with a regular polygon with multiple regular holes in the middle
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygonWithHoles) {
    // Construct the first 3x3 square hole
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{5, 6},
        BURST::geometry::Point2D{5, 9},
        BURST::geometry::Point2D{2, 9},
        BURST::geometry::Point2D{2, 6}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct the second 1x2 hole
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{6, 1},
        BURST::geometry::Point2D{6, 2},
        BURST::geometry::Point2D{4, 3},
        BURST::geometry::Point2D{4, 1}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the middle holes inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole1,
        *hole2
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon with multiple holes in the middle, but got nullopt.";
}

// Test for intended non-degeneracy with a regular polygon with a simple polygon hole in the middle
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygonWithSimpleHole) {
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon with a simple hole in the middle, but got nullopt.";
}

// Test for intended non-degeneracy with a regular polygon with a hole adjacent to the border
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygonWithBorderingHole) {
    // Construct the 2x1 hole that borders the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{3, 0},
        BURST::geometry::Point2D{3, 1},
        BURST::geometry::Point2D{1, 1},
        BURST::geometry::Point2D{1, 0}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the hole adjacent to the border
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon with a hole adjacent to the border, but got nullopt.";
}

// Test for intended non-degeneracy with a regular polygon with a hole sharing a corner with the outer boundary
TEST(WallSpaceConstructionTest, NonDegenerateRegularPolygonWithHoleOnCorner) {
    // Construct the square hole that shares a corner with the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{1, 0},
        BURST::geometry::Point2D{1, 1},
        BURST::geometry::Point2D{0, 1}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the hole sharing a corner with the outer boundary
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a regular polygon with a hole sharing a corner with the outer boundary, but got nullopt.";
}

// Test for intended non-degeneracy with a simple polygon with a hole in the middle
TEST(WallSpaceConstructionTest, NonDegenerateSimplePolygonWithHole) {
    // Construct a 4x2 hole
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{2, 4},
        BURST::geometry::Point2D{2, 6},
        BURST::geometry::Point2D{-2, 6},
        BURST::geometry::Point2D{-2, 4}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole";

    // Construct a WallSpace for a simple polygon with the hole inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a simple polygon with a hole in the middle, but got nullopt.";
}

// Test for intended non-degeneracy with a simple polygon with a hole adjacent to the border
TEST(WallSpaceConstructionTest, NonDegenerateSimplePolygonWithBorderingHole) {
    // Construct the 1x2 triangular hat borders the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{4, 8},
        BURST::geometry::Point2D{6, 8},
        BURST::geometry::Point2D{4, 12}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole";

    // Construct a WallSpace for a simple polygon with the hole adjacent to the border
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a simple polygon with a hole adjacent to the border, but got nullopt.";
}

// Test for intended non-degeneracy with a simple polygon with a hole sharing a corner with the outer boundary
TEST(WallSpaceConstructionTest, NonDegenerateSimplePolygonWithHoleOnCorner) {
    // Construct the 1x1 triangle hole that shares a corner with the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{2, -2},
        BURST::geometry::Point2D{2, 0},
        BURST::geometry::Point2D{0, 0}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole";

    // Construct a WallSpace for a simple polygon with the hole sharing a corner with the outer boundary
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    },
    {
        *hole
    });

    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    EXPECT_TRUE(wall_space.has_value()) << "Expected non-degenerate WallSpace for a simple polygon with a hole sharing a corner with the outer boundary, but got nullopt.";
}

// -- DEGENERATE NON-HOLED POLYGON TESTS ---------------------------------------

// Test for intended degeneracy with a straight line
TEST(WallSpaceConstructionTest, DegenerateStraightLine) {
    // Construct a WallSpace for a straight line
    // This is a degenerate polygon
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
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
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a normally valid polygon with out-of-order points, but got a valid geometry.";
}


// -- DEGENERATE HOLED POLYGON TESTS -------------------------------------------

// NOTE: Since a majority of these tests involve constructing degenerate holes, the BURST::geometry::construct_polygon function will not be used
// This is because that function is designed to return nullopt for degenerate polygons, which would prevent testing for degeneracy in the first place

// Test for intended degeneracy with a regular polygon with a single point hole in the middle
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithPointHole) {
    // Construct a single point hole in the middle
    // This is a degenerate hole
    BURST::geometry::Polygon2D hole = BURST::geometry::Polygon2D{};
    hole.push_back(BURST::geometry::Point2D{5, 5});

    // Construct a WallSpace for a square with the degenerate hole inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a single point hole in the middle, but got a valid geometry.";
}

// Test for intended degeneracy with a regular polygon with a linear hole in the middle
TEST(WallSpaceConstructionTste, DegenerateRegularPolygonWithLinearHole) {
    // Construct a linear hole in the middle
    // This is a degenerate hole
    BURST::geometry::Polygon2D hole = BURST::geometry::Polygon2D{};
    hole.push_back(BURST::geometry::Point2D{4, 4});
    hole.push_back(BURST::geometry::Point2D{6, 6});

    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a linear hole in the middle, but got a valid geometry.";
}

// Test for intended degeneracy with a regular polygon with a self-intersecting hole in the middle
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithSelfIntersectingHole) {
    // Construct a self-intersecting hole in the middle that looks like a bowtie
    // This is a degenerate hole
    BURST::geometry::Polygon2D hole = BURST::geometry::Polygon2D{};
    hole.push_back(BURST::geometry::Point2D{4, 4});
    hole.push_back(BURST::geometry::Point2D{6, 4});
    hole.push_back(BURST::geometry::Point2D{4, 6});
    hole.push_back(BURST::geometry::Point2D{6, 6});

    // Construct a WallSpace for a square with the degenerate hole inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a self-intersecting hole in the middle, but got a valid geometry.";
}

// Test for intended degeneracy with a regular polygon with multiple intersecting holes in the middle
// Each hole individually is non-degenerate
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithIntersectingHoles) {
    // Construct the first 2x2 square hole
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{5, 3},
        BURST::geometry::Point2D{5, 5},
        BURST::geometry::Point2D{3, 5},
        BURST::geometry::Point2D{3, 3}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct the second square 2x2 hole that intersects with the first hole
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{4, 2},
        BURST::geometry::Point2D{4, 4},
        BURST::geometry::Point2D{2, 4},
        BURST::geometry::Point2D{2, 2}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the intersecting holes inside
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole1,
        *hole2
    });

    // Expect the WallSpace with the degenerate holes to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with multiple intersecting holes in the middle, but got a valid geometry.";
}

// Construct a regular polygon with a hole that's larger than the outer boundary
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithEncapsulatingHole) {
    // Construct a 30x30 square hole that encapsulates the entire outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{-10, -10},
        BURST::geometry::Point2D{20, -10},
        BURST::geometry::Point2D{20, 20},
        BURST::geometry::Point2D{-10, 20}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the encapsulating hole
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a hole that's larger than the outer boundary, but got a valid geometry.";
}

// Construct a regular polygon with a hole that is identical to the outer boundary
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithIdenticalHole) {
    // Construct a hole that is identical to the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the identical hole
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a hole that is identical to the outer boundary, but got a valid geometry.";
}

// Construct a regular polygon with a hole that overlaps with the outer boundary
TEST(WallSpaceConstructionTest, DegenerateRegularPolygonWithOverlappingHole) {
    // Construct a hole that overlaps with the outer boundary
    std::optional<BURST::geometry::Polygon2D> hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{1, -1},
        BURST::geometry::Point2D{1, 1},
        BURST::geometry::Point2D{-1, 1},
        BURST::geometry::Point2D{-1, -1}
    });
    // Expect the hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole.";

    // Construct a WallSpace for a square with the overlapping hole
    std::optional<BURST::geometry::WallSpace> wall_space = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    },
    {
        *hole
    });

    // Expect the WallSpace with the degenerate hole to be degenerate
    // i.e., it is nullopt
    EXPECT_FALSE(wall_space.has_value()) << "Expected degenerate WallSpace for a regular polygon with a hole that overlaps with the outer boundary, but got a valid geometry.";
}
