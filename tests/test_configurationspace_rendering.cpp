#include <gtest/gtest.h>
#include <BURST/geometry.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/wall_space.hpp>
#include <BURST/renderable.hpp>

#include "test_helpers.hpp"

// Utility includes for tests

// Test for rendering a regular polygon and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderRegularPolygon) {
    // Construct a WallSpace for a square
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a regular polygon";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a regular polygon";

    // Create a CGAL Graphics Scene and render the WallSpace and ConfigurationSpace
    BURST::renderable::Scene scene;
    wall_space->render(scene);
    configuration_space->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    SUCCEED() << "If you're seeing this, something has gone terribly wrong";
}

// Test for rendering a simple polygon and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderSimplePolygon) {
    // Construct a WallSpace for a simple polygon
    // In this case, we'll use a concave polygon with an arrowhead shape
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    });

    // Expect the WallSpace to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a simple polygon";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a simple polygon";

    // Create a CGAL Graphics Scene and render the WallSpace and ConfigurationSpace 
    BURST::renderable::Scene scene;
    wall_space->render(scene);
    configuration_space->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);
    
    // If we reach this point without crashing, the test is successful
    SUCCEED() << "If you're seeing this, something has gone terribly wrong";
}

// Test for rendering a regular polygon with holes in the middle and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderRegularPolygonWithHoles) {
    // Construct the first 3x3 square hole
    std::optional<BURST::geometry::Polygon2D> hole1 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{5, 6},
        BURST::geometry::Point2D{5, 9},
        BURST::geometry::Point2D{2, 9},
        BURST::geometry::Point2D{2, 6}
    });
    // Expect the first hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole1.has_value()) << "Failed to construct non-degenerate first hole.";

    // Construct the second 1x2 hole
    std::optional<BURST::geometry::Polygon2D> hole2 = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{6, 1},
        BURST::geometry::Point2D{6, 2},
        BURST::geometry::Point2D{4, 2},
        BURST::geometry::Point2D{4, 1}
    });
    // Expect the second hole polygon to be non-degenerate
    // i.e., it is not nullopt
    ASSERT_TRUE(hole2.has_value()) << "Failed to construct non-degenerate second hole.";

    // Construct a WallSpace for a square with the middle holes inside
    auto wall_space = TestWallSpace::create({
        BURST::geometry::Point2D{0, 20},
        BURST::geometry::Point2D{-20, -20},
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{20, -20}
    },
    {
        *hole1,
        *hole2
    });
    // Expect the WallSpace with holes to be non-degenerate and valid
    // i.e., it is not nullopt
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a regular polygon with multiple holes in the middle";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a regular polygon with multiple holes in the middle";

    // Create a CGAL Graphics Scene and render the WallSpace and ConfigurationSpace
    BURST::renderable::Scene scene;
    wall_space->render(scene);
    configuration_space->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);

    // If we reach this point without crashing, the test is successful
    SUCCEED() << "If you're seeing this, something has gone terribly wrong";
}

// Test for rendering a simple polygon with a hole and its ConfigurationSpace
TEST(ConfigurationSpaceRenderingTest, RenderSimplePolygonWithHole) {
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
    auto wall_space = TestWallSpace::create({
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
    ASSERT_TRUE(wall_space.has_value()) << "Failed to construct non-degenerate WallSpace for a simple polygon with a hole in the middle";

    // Construct a ConfigurationSpace for a robot with radius 1
    auto configuration_space = wall_space->testConstructConfigurationSpace(1);
    // Expect the ConfigurationSpace to be non-degenerate
    // i.e., it is not nullptr
    ASSERT_NE(configuration_space, nullptr) << "Failed to construct non-degenerate ConfigurationSpace for a simple polygon with a hole in the middle";

    // Create a CGAL Graphics Scene and render the WallSpace and ConfigurationSpace
    BURST::renderable::Scene scene;
    wall_space->render(scene);
    configuration_space->render(scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);

    // If we reach this point without crashing, the test is successful
    SUCCEED() << "If you're seeing this, something has gone terribly wrong";
}
