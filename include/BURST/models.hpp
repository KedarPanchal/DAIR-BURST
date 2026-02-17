#ifndef MODELS_HPP
#define MODELS_HPP

#include <random>
#include <optional>

#include "types.hpp"
#include "configuration_geometry.hpp"

using ConfigurationGeometry = BURST::geometry::ConfigurationGeometry;

namespace BURST::models {
    
    /*
     * RotationModel classes define how the robot's rotation is affected by noise.
     */
    class RotationModel {
    public:
        virtual fscalar operator() (fscalar angle) const = 0;

        virtual fscalar getMaxRotation(fscalar angle) const = 0;
        virtual fscalar getMinRotation(fscalar angle) const = 0;
    };
    
    // Pseudorandom rotation model
    template <typename PRNG = std::mt19937>
    class PRRotationModel : public RotationModel {
    private:
        fscalar max_rotation_error;
        mutable PRNG prng;
        mutable std::uniform_real_distribution<double> rand_dist; // Generate from -1 to 1 to scale max_rotation_error by

    public:
        PRRotationModel(fscalar max_rotation_error) : max_rotation_error{max_rotation_error}, prng{std::random_device{}()}, rand_dist{-1.0, 1.0} {}

        fscalar operator() (fscalar angle) const override {
            // Generate a random rotation error scaled by max_rotation_error
            return angle + this->rand_dist(this->prng) * max_rotation_error;
        }

        fscalar getMaxRotation(fscalar angle) const override {
            return angle + this->max_rotation_error;
        }
        fscalar getMinRotation(fscalar angle) const override {
            return angle - this->max_rotation_error;
        }
    };
    
    // Seeded pseudorandom rotation model
    template <typename PRNG = std::mt19937>
    class SeededPRRotationModel : public RotationModel {
    private:
        fscalar max_rotation_error;
        unsigned int seed;
        mutable PRNG prng;
        mutable std::uniform_real_distribution<double> rand_dist; // Generate from -1 to 1 to scale max_rotation_error by
    
    public:
        SeededPRRotationModel(fscalar max_rotation_error, unsigned int seed) : max_rotation_error{max_rotation_error}, seed{seed}, prng{seed}, rand_dist{-1.0, 1.0} {}

        fscalar operator() (fscalar angle) const override {
            // Generate a random rotation error scaled by max_rotation_error
            return angle + this->rand_dist(this->prng) * max_rotation_error;
        }

        fscalar getMaxRotation(fscalar angle) const override {
            return angle + this->max_rotation_error;
        }
        fscalar getMinRotation(fscalar angle) const override {
            return angle - this->max_rotation_error;
        }
    };
    
    // Fixed-value rotation model
    class FixedRotationModel : public RotationModel {
    private:
        fscalar max_rotation_error;
        fscalar fixed_rotation;
    
    public:

        FixedRotationModel(fscalar max_rotation_error, fscalar fixed_rotation_scale = 1) : max_rotation_error{max_rotation_error}, fixed_rotation{fixed_rotation_scale * max_rotation_error} {}

        fscalar operator() (fscalar angle) const override {
            // Apply a fixed rotation error scaled by fixed_rotation_scale
            return angle + this->fixed_rotation;
        }

        fscalar getMaxRotation(fscalar angle) const override {
            return angle + this->max_rotation_error;
        }
        fscalar getMinRotation(fscalar angle) const override {
            return angle - this->max_rotation_error;
        }
    };
    
    /*
     * MovementModel classes define how the robot's movement is affected by noise.
     */
    class MovementModel {
    public:
        virtual Point_2 operator() (const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const = 0;
        virtual Segment_2 generateTrajectory(const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const = 0;
    };
    
    // Linear trajectory movement model
    class LinearMovementModel : public MovementModel {
    public:
        Point_2 operator() (const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const override;
        Segment_2 generateTrajectory(const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const override;
    };
    
}
#endif
