#include <gtest/gtest.h>

#include <BURST/robot.hpp>
#include <BURST/wall_space.hpp>
#include <BURST/configuration_space.hpp>
#include <BURST/geometry.hpp>
#include <BURST/renderable.hpp>

#include <optional>

// -- TEST FIXTURE SETUP -------------------------------------------------------
class RobotRenderingTest : public ::testing::Test {
protected:
    std::optional<BURST::geometry::WallSpace> wall_space;
    std::shared_ptr<BURST::geometry::ConfigurationSpace> config_space;
    std::optional<BURST::Robot<>> robot;

    void SetUp() override {
        // Construct a wallSpace for a square with a hole in the middle
        // Construct the hole polygon
        std::optional<BURST::geometry::Polygon2D> hole_polygon = BURST::geometry::construct_polygon({
            BURST::geometry::Point2D{4, 4},
            BURST::geometry::Point2D{6, 4},
            BURST::geometry::Point2D{6, 6},
            BURST::geometry::Point2D{4, 6}
        });
        ASSERT_TRUE(hole_polygon.has_value()) << "Failed to construct hole polygon";
        // Construct the wall space with the hole
        this->wall_space = BURST::geometry::WallSpace::create({
            BURST::geometry::Point2D{0, 0},
            BURST::geometry::Point2D{10, 0},
            BURST::geometry::Point2D{10, 10},
            BURST::geometry::Point2D{0, 10}
        },
        {
            *hole_polygon
        });
        ASSERT_TRUE(wall_space.has_value()) << "Failed to construct wall space";

        // Construct a robot
        this->robot = BURST::Robot<>::create(
            1.0, 
            BURST::geometry::Point2D{1, 1},
            0.1,
            42
        );
        ASSERT_TRUE(robot.has_value()) << "Failed to construct robot with valid parameters and seed";

        // Generate a configuration space for the robot
        bool result = this->wall_space->generateConfigurationSpace(*this->robot);
        ASSERT_TRUE(result) << "Failed to generate configuration space for robot";
        this->config_space = this->robot->getConfigurationEnvironmentPtr();
    }
};

// -- ROBOT RENDERING TESTS ----------------------------------------------------

// Test that a robot can be rendered
TEST_F(RobotRenderingTest, Render) {
    // Create a CGAL Graphics Scene to render the wall, robot, and configuration space
    BURST::renderable::Scene scene;

    // Render all elements
    BURST::renderable::render_all({
        &*this->wall_space,
        this->config_space.get(),
        &*this->robot
    }, scene);

    // Draw the scene in a CGAL viewer
    CGAL::draw_graphics_scene(scene);

    // If we reach this point without crashing, the test is successful
    SUCCEED() << "If you're seeing this, something has gone terribly wrong";
}

// -- STADIUM RENDERING TESTS --------------------------------------------------
