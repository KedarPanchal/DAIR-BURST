#include <gtest/gtest.h>
#include <BURST/configuration_geometry.hpp>
#include <BURST/wall_geometry.hpp>

#include "test_wallgeometry_class.hpp"

// Utility includes for tests
#include <BURST/types.hpp>

// Test for rendering a regular polygon and its configuration geometry
TEST(ConfigurationGeometryRenderingTest, RenderRegularPolygon) {
    // Construct a WallGeometry for a square
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 0),
        BURST::Point_2(10, 0),
        BURST::Point_2(10, 10),
        BURST::Point_2(0, 10)
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon";

    // Construct a configuration geometry for a robot with radius 1
    auto configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry for a regular polygon";

    // Create a CGAL Graphics Scene and render the WallGeometry and ConfigurationGeometry
    BURST::scene scene;
    configuration_geometry->render(scene);
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}

// Test for rendering a simple polygon and its configuration geometry
TEST(ConfigurationGeometryRenderingTest, RenderSimplePolygon) {
    // Construct a WallGeometry for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_geometry = TestWallGeometry::create({
        BURST::Point_2(0, 20),
        BURST::Point_2(-20, -20),
        BURST::Point_2(0, 0),
        BURST::Point_2(20, -20)
    });

    // Expect the WallGeometry to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a simple polygon";

    // Construct a configuration geometry for a robot with radius 1
    auto configuration_geometry = wall_geometry->testConstructConfigurationGeometry(1);
    // Expect the configuration geometry to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_geometry, nullptr) << "Failed to construct non-degenerate ConfigurationGeometry for a simple polygon";

    // Create a CGAL Graphics Scene and render the WallGeometry and ConfigurationGeometry
    BURST::scene scene;
    configuration_geometry->render(scene);
    wall_geometry->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    EXPECT_TRUE(true) << "If you're seeing this, something has gone terribly wrong, and EXPECT_TRUE(true) is a LIE";
}
