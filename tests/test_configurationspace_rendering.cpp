#include <gtest/gtest.h>
#include <BURST/geometric_types.hpp>
#include <BURST/graphics_types.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>

#include "test_helpers.hpp"

// Utility includes for tests

// Test for rendering a regular polygon and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderRegularPolygon) {
    GTEST_SKIP() << "Rendering tests are currently disabled";
    // Construct a WallSpace for a square
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a regular polygon";

    // Create a CGAL Graphics Scene and render the WallGeometry and ConfigurationSpace
    BURST::graphics::Scene scene;
    configuration_geometry->render(scene);
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer -- TODO: Re-enable once rendering import issues are fixed as this currently crashes the build
    // CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}

// Test for rendering a simple polygon and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderSimplePolygon) {
    GTEST_SKIP() << "Rendering tests are currently disabled";
    // Construct a WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = TestWallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a simple polygon";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_geometry = wall_geometry->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a simple polygon";

    // Create a CGAL Graphics Scene and render the WallSpace and ConfigurationSpace 
    BURST::graphics::Scene scene;
    configuration_geometry->render(scene);
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer -- TODO: Re-enable once rendering import issues are fixed as this currently crashes the build
    // CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}
