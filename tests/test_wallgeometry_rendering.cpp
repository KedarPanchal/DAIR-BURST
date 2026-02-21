#include <gtest/gtest.h>
#include <BURST/wall_geometry.hpp>

// Utility includes for tests
#include <BURST/types.hpp>

// Test for rendering a regular polygon
TEST(WallGeometryRenderingTest, RenderRegularPolygon) {
    // Construct a WallGeometry for a square
    auto wall_geometry = BURST::geometry::WallGeometry::create({
        BURST::Point2D{0, 0},
        BURST::Point2D{10, 0},
        BURST::Point2D{10, 10},
        BURST::Point2D{0, 10}
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon";
    
    // Create a CGAL Graphics Scene and render the WallGeometry
    BURST::scene scene;
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);

    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}

// Test for rendering a simple polygon
TEST(WallGeometryRenderingTest, RenderSimplePolygon) {
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
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a simple polygon";

    // Create a CGAL Graphics Scene and render the WallGeometry
    BURST::scene scene;
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);

    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}
