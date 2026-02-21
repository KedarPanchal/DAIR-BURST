#include <gtest/gtest.h>
#include <BURST/models.hpp>

// Utility includes for tests

// Test a rotation model generating an unseeded random rotation
TEST(RotationModelTest, UnseededRandomRotation) {
    // Construct a RotationModel with a max rotation error of 0.5 radians
    // and empty seed to use random_device
    auto rotation_model = BURST::models::RotationModel<>(0.5);

    // Generate a random rotation for an angle of 1.0 radians
    BURST::numeric::fscalar rotated_angle = rotation_model(1.0);

    EXPECT_TRUE(rotated_angle >= 0.5 && rotated_angle <= 1.5) << "Expected rotated angle to be within the range of [0.5, 1.5], but got " << CGAL::to_double(rotated_angle);
}

// Test a rotation model generating a seeded random rotation
TEST(RotationModelTest, SeededRandomRotation) {
    // Construct a RotationModel with a max rotation error of 0.5 radians and a fixed seed
    auto rotation_model = BURST::models::RotationModel<>(0.5, 42);

    // Generate a random rotation for an angle of 1.0 radians
    BURST::numeric::fscalar rotated_angle = rotation_model(1.0);

    EXPECT_TRUE(rotated_angle >= 0.5 && rotated_angle <= 1.5) << "Expected rotated angle to be within the range of [0.5, 1.5], but got " << CGAL::to_double(rotated_angle);
}

// Test a rotation model with a flat distribution that generates the same rotation every time
TEST(RotationModelTest, FlatDistributionRotation) {
    // Construct a RotationModel with a max rotation error of 0.5 radians, a fixed seed, and a flat distribution
    auto rotation_model = BURST::models::MaximumRotationModel(0.5);

    // Generate a random rotation for an angle of 1.0 radians
    BURST::numeric::fscalar rotated_angle = rotation_model(1.0);

    EXPECT_EQ(rotated_angle, 1.5) << "Expected rotated angle to be 1.5 with a flat distribution, but got " << CGAL::to_double(rotated_angle);
}

// Test a rotation model with a flat distribution that generates the same rotation every time for multiple calls
TEST(RotationModelTest, FlatDistributionRotationConsistency) {
    // Construct a RotationModel with a max rotation error of 0.5 radians, a fixed seed, and a flat distribution
    auto rotation_model = BURST::models::MaximumRotationModel(0.5);

    // Generate multiple random rotations for an angle of 1.0 radians
    for (int i = 0; i < 10; i++) {
        BURST::numeric::fscalar rotated_angle = rotation_model(1.0);
        EXPECT_EQ(rotated_angle, 1.5) << "Expected rotated angle to be 1.5 with a flat distribution, but got " << CGAL::to_double(rotated_angle) << " on iteration " << i;
    }
}
