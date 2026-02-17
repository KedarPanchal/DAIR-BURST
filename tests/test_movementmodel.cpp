#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests
#include "test_helpers.hpp"
#include <optional>

// Define a test fixture containing a valid ConfigurationGeometry for testing the MovementModel
// This should be reused across multiple tests for the MovementModel to keep the geometry consistent
class MovementModelTest : public ::testing::Test {
protected:
    std::unique_ptr<BURST::geometry::ConfigurationGeometry> config_geometry;

    void SetUp() override {
        // Construct a TestWallGeometry for a square and generate a ConfigurationGeometry with a robot radius of 1
        auto wall_geometry = TestWallGeometry::create({
            BURST::Point_2(0, 0),
            BURST::Point_2(10, 0),
            BURST::Point_2(10, 10),
            BURST::Point_2(0, 10)
        });
        // Expect the WallGeometry to be non-degenerate
        // i.e. it is not nullopt
        ASSERT_TRUE(wall_geometry.has_value()) << "Failed to construct non-degenerate WallGeometry for a regular polygon in test fixture setup";

        // Construct a configuration geometry for a robot with radius 1
        this->config_geometry = wall_geometry->testConstructConfigurationGeometry(1.0);
        ASSERT_NE(this->config_geometry, nullptr) << "Failed to construct ConfigurationGeometry from WallGeometry in test fixture setup";
    }
};
