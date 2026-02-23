#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/graphics_types.hpp>
#include <BURST/wall_space.hpp>

// Utility includes for tests

// Test for rendering a regular polygon
TEST(WallSpaceRenderingTest, RenderRegularPolygon) {
    // Construct a WallSpace for a square
    auto wall_geometry = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace for a regular polygon";
    
    // Create a CGAL Graphics Scene and render the WallSpace
    BURST::graphics::Scene scene;
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer -- TODO: Re-enable once rendering import issues are fixed
    // CGAL::draw_graphics_scene(scene);
    FAIL() << "Rendering tests are currently disabled";

    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}

// Test for rendering a simple polygon
TEST(WallSpaceRenderingTest, RenderSimplePolygon) {
    // Construct a WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = BURST::geometry::WallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallSpace for a simple polygon";

    // Create a CGAL Graphics Scene and render the WallSpace
    BURST::graphics::Scene scene;
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer -- TODO: Re-enable once rendering import issues are fixed
    // CGAL::draw_graphics_scene(scene);
    FAIL() << "Rendering tests are currently disabled";

    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}
