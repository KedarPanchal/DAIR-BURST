#include <gtest/gtest.h>

#include <BURST/geometry.hpp>
#include <BURST/renderable.hpp>

#include <optional>

// -- GEOMETRY::RENDERABLE (POLYGON / SET) TESTS --------------------------------

TEST(MiscellaneousRenderingTest, RenderLinearPolygonRenderable) {
    // Construct a simple square polygon
    auto polygon = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    ASSERT_TRUE(polygon.has_value()) << "Failed to construct non-degenerate linear polygon.";

    // Create a CGAL Graphics Scene and render the polygon wrapper
    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{255, 0, 0};
    auto r = BURST::geometry::renderable(*polygon, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

TEST(MiscellaneousRenderingTest, RenderLinearPolygonSetRenderable) {
    // Construct a simple square polygon
    auto polygon = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    ASSERT_TRUE(polygon.has_value()) << "Failed to construct non-degenerate linear polygon.";

    // Convert to a polygon set and render
    BURST::geometry::LinearPolygonSet2D polygon_set{*polygon};
    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{0, 255, 0};
    auto r = BURST::geometry::renderable(polygon_set, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

TEST(MiscellaneousRenderingTest, RenderCurvilinearPolygonRenderable) {
    // Construct a circle as a curvilinear polygon
    auto circle = BURST::geometry::construct_circle(5, BURST::geometry::Point2D{0, 0});
    ASSERT_TRUE(circle.has_value()) << "Failed to construct non-degenerate curvilinear circle polygon.";

    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{0, 0, 255};
    auto r = BURST::geometry::renderable(*circle, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

TEST(MiscellaneousRenderingTest, RenderCurvilinearPolygonSetRenderable) {
    // Construct a circle as a curvilinear polygon
    auto circle = BURST::geometry::construct_circle(5, BURST::geometry::Point2D{0, 0});
    ASSERT_TRUE(circle.has_value()) << "Failed to construct non-degenerate curvilinear circle polygon.";

    // Convert to a curvilinear polygon set and render
    BURST::geometry::CurvilinearPolygonSet2D polygon_set{*circle};
    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{255, 0, 255};
    auto r = BURST::geometry::renderable(polygon_set, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

// -- GEOMETRY::RENDERABLE (HOLED POLYGON) TESTS --------------------------------

TEST(MiscellaneousRenderingTest, RenderHoledLinearPolygonRenderable) {
    // Outer boundary: 10x10 square
    auto outer = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{0, 0},
        BURST::geometry::Point2D{10, 0},
        BURST::geometry::Point2D{10, 10},
        BURST::geometry::Point2D{0, 10}
    });
    ASSERT_TRUE(outer.has_value()) << "Failed to construct non-degenerate outer boundary polygon.";

    // Hole: 2x2 square in the middle (reverse to clockwise, matching library convention)
    auto hole = BURST::geometry::construct_polygon({
        BURST::geometry::Point2D{4, 4},
        BURST::geometry::Point2D{6, 4},
        BURST::geometry::Point2D{6, 6},
        BURST::geometry::Point2D{4, 6}
    });
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole polygon.";
    BURST::geometry::Polygon2D oriented_hole = *hole;
    if (oriented_hole.orientation() != CGAL::CLOCKWISE) oriented_hole.reverse_orientation();

    // Build a holed polygon
    BURST::geometry::HoledPolygon2D holed{*outer};
    holed.add_hole(oriented_hole);

    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{255, 255, 0};
    auto r = BURST::geometry::renderable(holed, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

TEST(MiscellaneousRenderingTest, RenderHoledCurvilinearPolygonRenderable) {
    // Outer boundary: circle radius 10, hole: circle radius 3 (same center)
    auto outer = BURST::geometry::construct_circle(10, BURST::geometry::Point2D{0, 0});
    ASSERT_TRUE(outer.has_value()) << "Failed to construct non-degenerate outer boundary circle.";
    auto hole = BURST::geometry::construct_circle(3, BURST::geometry::Point2D{0, 0});
    ASSERT_TRUE(hole.has_value()) << "Failed to construct non-degenerate hole circle.";

    BURST::geometry::HoledCurvilinearPolygon2D holed{*outer};
    holed.add_hole(*hole);

    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{0, 255, 255};
    auto r = BURST::geometry::renderable(holed, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

// -- GEOMETRY::RENDERABLE (ARRANGEMENT) TESTS ----------------------------------

TEST(MiscellaneousRenderingTest, RenderSegmentArrangementRenderable) {
    // Two segments that cross to create a non-trivial arrangement
    const BURST::geometry::Segment2D s1{
        BURST::geometry::Point2D{-5, 0},
        BURST::geometry::Point2D{5, 0}
    };
    const BURST::geometry::Segment2D s2{
        BURST::geometry::Point2D{0, -5},
        BURST::geometry::Point2D{0, 5}
    };

    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{128, 128, 128};
    auto r = BURST::geometry::renderable(s1, s2, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}

TEST(MiscellaneousRenderingTest, RenderCurveArrangementRenderable) {
    // Build curves via geometry::construct_curve from segments to avoid relying on CGAL arc constructors.
    const BURST::geometry::Segment2D s1{
        BURST::geometry::Point2D{-5, -5},
        BURST::geometry::Point2D{5, 5}
    };
    const BURST::geometry::Segment2D s2{
        BURST::geometry::Point2D{-5, 5},
        BURST::geometry::Point2D{5, -5}
    };
    const BURST::geometry::MonotoneCurve2D c1 = BURST::geometry::construct_curve(s1);
    const BURST::geometry::MonotoneCurve2D c2 = BURST::geometry::construct_curve(s2);

    BURST::renderable::Scene scene;
    const BURST::renderable::Color color{200, 100, 50};
    auto r = BURST::geometry::renderable(c1, c2, scene, color);

    EXPECT_EQ(r->defaultColor(), color) << "Renderable default color should match the provided color.";
    r->render(scene, r->defaultColor());

    CGAL::draw_graphics_scene(scene);
    SUCCEED() << "If you're seeing this, something has gone terribly wrong.";
}
