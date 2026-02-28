#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>

#include "test_helpers.hpp"

// Utility includes for tests
#include <optional>
#include <memory>
#include <vector>
#include <iterator>

// -- SIMPLE POLYGON INTERSECTION TESTS ----------------------------------------

// Create a test fixture for ConfigurationSpace intersection tests with a regular polygon
class ConfigurationSpaceRegularPolygonIntersectionTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_space;

    void SetUp() override {
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
        this->configuration_space = wall_space->testConstructConfigurationSpace(1);
        // Expect the ConfigurationSpace to be non-degenerate
        // i.e. it is not nullptr
        ASSERT_NE(this->configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";
    }
};

// Test point intersection for a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, PointIntersectionRegularPolygon) {
    // Create a point that lies on the edge of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{1, 5};

    // Expect the point to intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is not nullopt
    EXPECT_TRUE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to intersect with the ConfigurationSpace, but got nullopt";
}

// Test point intersection at a corner for a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, PointIntersectionAtCornerRegularPolygon) {
    // Create a point that lies on the corner of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{1, 1};

    // Expect the point to intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is not nullopt
    EXPECT_TRUE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to intersect with the ConfigurationSpace, but got nullopt";
}

// Test invalid point intersection on the interior of a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, InvalidPointIntersectionInteriorRegularPolygon) {
    // Create a point that lies not on the boundary of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{5, 5};

    // Expect the point to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is nullopt
    EXPECT_FALSE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to not intersect with the ConfigurationSpace, but got a valid intersection";
}

// Test invalid point intersection on the exterior of a ConfigurationSpace with a regular polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, InvalidPointIntersectionExteriorRegularPolygon) {
    // Create a point that lies not on the boundary of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{15, 5};

    // Expect the point to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is nullopt
    EXPECT_FALSE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to not intersect with the ConfigurationSpace, but got a valid intersection";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts on the edge of the ConfigurationSpace and points inward
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionRegularPolygon) {
    // Create a ray that starts on the edge of the ConfigurationSpace and points inward
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{1, 5}, BURST::geometry::Vector2D{1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}  

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts at the corner of the ConfigurationSpace and points inward along the angle bisector of the corner
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionAtCornerRegularPolygon) {
    // Create a ray that starts at the corner of the ConfigurationSpace and points inward along the angle bisector of the corner
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{1, 1}, BURST::geometry::Vector2D{1, 1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once, since it should intersect only at a corner
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts interior to the ConfigurationSpace
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionInteriorRegularPolygon) {
    // Create a ray that starts at the interior of the ConfigurationSpace and points to an edge
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{5, 5}, BURST::geometry::Vector2D{1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once, since it should only intersect with the edge that the ray points towards
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts exterior to the ConfigurationSpace
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionExteriorRegularPolygon) {
    // Create a ray that starts at the exterior of the ConfigurationSpace and points towards the ConfigurationSpace
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{-100, 5}, BURST::geometry::Vector2D{1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly twice, since it should intersect with both the top and bottom edges of the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection == 2
    EXPECT_EQ(intersection_count, 2) << "Expected ray to intersect with the ConfigurationSpace exactly twice, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts on the edge of the ConfigurationSpace and points outward
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionOutwardRegularPolygon) {
    // Create a ray that starts at the edge of the ConfigurationSpace and points outside the ConfigurationSpace
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{1, 5}, BURST::geometry::Vector2D{-1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection == 0
    EXPECT_EQ(intersection_count, 0) << "Expected ray to not intersect with the ConfigurationSpace, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts on the corner of the ConfigurationSpace and points outward
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionOutwardAtCornerRegularPolygon) {
    // Create a ray that starts at the corner of the ConfigurationSpace and points outside the ConfigurationSpace
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{1, 1}, BURST::geometry::Vector2D{-1, -1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection == 0
    EXPECT_EQ(intersection_count, 0) << "Expected ray to not intersect with the ConfigurationSpace, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a regular polygon with a ray that starts at the exterior of the ConfigurationSpace and points outward
TEST_F(ConfigurationSpaceRegularPolygonIntersectionTest, RayIntersectionOutwardExteriorRegularPolygon) {
    // Create a ray that starts at the exterior of the ConfigurationSpace and points outside the ConfigurationSpace
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{15, 5}, BURST::geometry::Vector2D{1, 0}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to not intersect with the ConfigurationSpace 
    // i.e., ConfigurationSpace::intersection == 0
    EXPECT_EQ(intersection_count, 0) << "Expected ray to not intersect with the ConfigurationSpace, but got " << intersection_count << " intersections";
}


// -- CONCAVE POLYGON INTERSECTION TESTS ---------------------------------------

// Create a test fixture for ConfigurationSpace intersection tests with a simple, but concave polygon
class ConfigurationSpaceConcavePolygonIntersectionTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_space;

    void SetUp() override {
        auto wall_space = TestWallSpace::create({
            // Construct an already known non-degenerate WallSpace for a simple polygon
            // In this case, we'll use a concave polygon with an arrowhead shape
            BURST::geometry::Point2D{0, 20},
            BURST::geometry::Point2D{-20, -20},
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{20, -20}
        });
        // Expect the WallSpace to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace";

        // Construct a ConfigurationSpace for a robot with radius 1
        this->configuration_space = wall_space->testConstructConfigurationSpace(1);
        // Expect the ConfigurationSpace to be non-degenerate
        // i.e. it is not nullptr
        ASSERT_NE(this->configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace";
    }
};

// Test point intersection for a ConfigurationSpace with a concave polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, PointIntersectionConcavePolygon) {
    // Create a point that lies on the edge of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{0, 1};

    // Expect the point to intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is not nullopt
    EXPECT_TRUE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to intersect with the ConfigurationSpace, but got nullopt";
}

// Test invalid point intersection on the interior of a ConfigurationSpace with a concave polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, InvalidPointIntersectionConcavePolygon) {
    // Create a point that lies not on the boundary of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{0, 5};

    // Expect the point to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is nullopt
    EXPECT_FALSE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to not intersect with the ConfigurationSpace, but got a valid intersection";
}

// Test invalid point intersection on the exterior of a ConfigurationSpace with a concave polygon
// TODO: Once the point intersection function is updated to return an edge, update this test case accordingly
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, InvalidPointIntersectionExteriorConcavePolygon) {
    // Create a point that lies not on the boundary of the ConfigurationSpace
    BURST::geometry::Point2D intersection_point{0, 100};

    // Expect the point to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection is nullopt
    EXPECT_FALSE(this->configuration_space->intersection(intersection_point).has_value()) << "Expected point to not intersect with the ConfigurationSpace, but got a valid intersection";
}

// Test ray intersection for a ConfigurationSpace with a concave polygon with a ray that starts on the edge of the ConfigurationSpace and points inward
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, RayIntersectionConcavePolygon) {
    // Create a ray that starts on the edge of the ConfigurationSpace and points inward
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{0, 1}, BURST::geometry::Vector2D{0, 1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a concave polygon that starts at the interior of the ConfigurationSpace and points inward
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, RayIntersectionInteriorConcavePolygon) {
    // Create a ray that starts at the interior of the ConfigurationSpace and points inward
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{0, 5}, BURST::geometry::Vector2D{0, -1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly once
    // i.e., ConfigurationSpace::intersection == 1
    EXPECT_EQ(intersection_count, 1) << "Expected ray to intersect with the ConfigurationSpace exactly once, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a concave polygon that starts at the exterior of the ConfigurationSpace and points inward
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, RayIntersectionExteriorConcavePolygon) {
    // Create a ray that starts at the exterior of the ConfigurationSpace and points inward
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{0, 100}, BURST::geometry::Vector2D{0, -1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to intersect with the ConfigurationSpace exactly twice, since it should intersect with both the top and bottom edges of the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection == 2
    EXPECT_EQ(intersection_count, 2) << "Expected ray to intersect with the ConfigurationSpace exactly twice, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a concave polygon with a ray that starts on the edge of the ConfigurationSpace and points outward
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, RayIntersectionOutwardConcavePolygon) {
    // Create a ray that starts at the edge of the ConfigurationSpace and points outside the Configuration
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{0, 1}, BURST::geometry::Vector2D{0, -1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to not intersect with the ConfigurationSpace
    // i.e., ConfigurationSpace::intersection == 0
    EXPECT_EQ(intersection_count, 0) << "Expected ray to not intersect with the ConfigurationSpace, but got " << intersection_count << " intersections";
}

// Test ray intersection for a ConfigurationSpace with a concave polygon with a ray that starts at the exterior of the ConfigurationSpace and points outward
TEST_F(ConfigurationSpaceConcavePolygonIntersectionTest, RayIntersectionOutwardExteriorConcavePolygon) {
    // Create a ray that starts at the exterior of the ConfigurationSpace and points outside the Configuration
    BURST::geometry::Ray2D ray{BURST::geometry::Point2D{0, 100}, BURST::geometry::Vector2D{0, 1}};
    // Create a collection for the intersections
    std::vector<BURST::geometry::Point2D> intersections;

    // Compute the intersections of the ray with the ConfigurationSpace
    size_t intersection_count = this->configuration_space->intersection<BURST::geometry::Ray2D, BURST::geometry::Segment2D>(ray, std::back_inserter(intersections));

    // Expect the ray to not intersect with the ConfigurationSpace 
    // i.e., ConfigurationSpace::intersection == 0
    EXPECT_EQ(intersection_count, 0) << "Expected ray to not intersect with the ConfigurationSpace, but got " << intersection_count << " intersections";
}
