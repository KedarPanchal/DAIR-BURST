#include <gtest/gtest.h>
#include <BURST/configuration_geometry.hpp>
#include <BURST/wall_geometry.hpp>

#include "test_helpers.hpp"

// Utility includes for tests

// Test for an intended point intersection on the configuration geometry
TEST(ConfigurationGeometryIntersectionTest, PointIntersection) {
    // Construct an already known non-degenerate WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 0),
        BURST::Point_2(10, 0),
        BURST::Point_2(10, 10),
        BURST::Point_2(0, 10)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // For some reason if the ConfigurationGeometry is degenerate, crash the test
    ASSERT_NE(config_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry";

    // Create a point on the edge of a configuration geometry
    BURST::Point_2 intersection_point(5, 1);

    // Find the intersecting edge
    auto maybe_edge = config_geometry->getEdge(intersection_point);
    
    // Expect the edge to exist
    EXPECT_TRUE(maybe_edge.has_value()) << "Expected to find an edge containing the intersection point, but got nullopt";
    // Expect the edge to contain the intersection point
    if (maybe_edge.has_value()) {
        BURST::Segment_2 edge = maybe_edge.value();
        EXPECT_TRUE(edge.has_on(intersection_point)) << "Expected the edge to contain the intersection point, but it does not";
    }
}

// Test for an intended segment intersection on the configuration geometry
TEST(ConfigurationGeometryIntersectionTest, SegmentIntersection) {
    // Construct an already known non-degenerate WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 0),
        BURST::Point_2(10, 0),
        BURST::Point_2(10, 10),
        BURST::Point_2(0, 10)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // For some reason if the ConfigurationGeometry is degenerate, crash the test
    ASSERT_NE(config_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry";

    // Create a segment that overlaps with the bottom edge of the configuration geometry
    BURST::Segment_2 intersection_segment(BURST::Point_2(5, 1), BURST::Point_2(7, 1));

    // Find the intersecting edge
    auto maybe_edge = config_geometry->getEdge(intersection_segment);
    
    // Expect the edge to exist
    EXPECT_TRUE(maybe_edge.has_value()) << "Expected to find an edge containing the intersection segment, but got nullopt";
    // Expect the edge to contain the intersection segment
    if (maybe_edge.has_value()) {
        BURST::Segment_2 edge = maybe_edge.value();
        EXPECT_TRUE(edge.has_on(intersection_segment.source()) && edge.has_on(intersection_segment.target())) << "Expected the edge to contain the intersection segment, but it does not";
    }
}

// Test for an intended nonintersection of a point on the configuration geometry
TEST(ConfigurationGeometryIntersectionTest, PointNonIntersection) {
    // Construct an already known non-degenerate WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 0),
        BURST::Point_2(10, 0),
        BURST::Point_2(10, 10),
        BURST::Point_2(0, 10)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // For some reason if the ConfigurationGeometry is degenerate, crash the test
    ASSERT_NE(config_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry";

    // Create a point that is not on the bounds of the configuration geometry
    BURST::Point_2 non_intersection_point(5, 5);

    // Find the intersecting edge
    auto maybe_edge = config_geometry->getEdge(non_intersection_point);
    
    // Expect no edge to exist
    EXPECT_FALSE(maybe_edge.has_value()) << "Expected to find no edge containing the nonintersection point, but got an edge";
}

// Test for an intended nonintersection of a segment on the configuration geometry
TEST(ConfigurationGeometryIntersectionTest, SegmentNonIntersection) {
    // Construct an already known non-degenerate WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 0),
        BURST::Point_2(10, 0),
        BURST::Point_2(10, 10),
        BURST::Point_2(0, 10)
    });
    // For some reason if the WallGeometry is degenerate, crash the test
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry";

    // Construct a configuration geometry for a robot with radius 1
    auto config_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // For some reason if the ConfigurationGeometry is degenerate, crash the test
    ASSERT_NE(config_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry";

    // Create a segment that does not overlap with any edge of the configuration geometry
    BURST::Segment_2 non_intersection_segment(BURST::Point_2(5, 5), BURST::Point_2(7, 5));

    // Find the intersecting edge
    auto maybe_edge = config_geometry->getEdge(non_intersection_segment);

    // Expect no edge to exist
    EXPECT_FALSE(maybe_edge.has_value()) << "Expected to find no edge containing the nonintersection segment, but got an edge";
}
