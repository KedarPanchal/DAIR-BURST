#include <gtest/gtest.h>

#include <BURST/robot.hpp>
#include <BURST/geometric_types.hpp>

// -- TEST FIXTURE SETUP -------------------------------------------------------
class RobotTest : public ::testing::Test {
    BURST::Robot<> robot;
protected:
    void SetUp() override {
    }
};
