#ifndef MODELS_HPP
#define MODELS_HPP

#include <random>
#include <optional>

#include "types.hpp"
#include "configuration_geometry.hpp"

using ConfigurationGeometry = BURST::geometry::ConfigurationGeometry;

namespace BURST::models {
    
    // Internal implementations not intended for public use
    namespace detail {
        // Generates the same number for every RNG, which is useful for testing
        // This allows for templating the rotation model and avoiding inheritance
        class flat_distribution {
        public:
            template <typename RNG> double operator() (RNG& rng) const {
                return 1.0;
            }
        };
    }
    
    /*
     * RotationModel classes define how the robot's rotation is affected by noise.
     */
    template <typename PRNG = std::mt19937, typename Dist = std::uniform_real_distribution<double>>
    class RotationModel {
    private:
        fscalar max_rotation_error;
        mutable PRNG prng;
        mutable Dist rand_dist; // Generate from -1 to 1 to scale max_rotation_error by

    public:
        RotationModel(fscalar max_rotation_error, unsigned int seed = std::random_device{}()) : max_rotation_error{max_rotation_error}, prng{seed}, rand_dist{-1.0, 1.0} {}

        fscalar operator() (fscalar angle) const {
            // Generate a random rotation error scaled by max_rotation_error
            return angle + this->rand_dist(this->prng) * max_rotation_error;
        }

        fscalar getMaxRotation(fscalar angle) const {
            return angle + this->max_rotation_error;
        }
        fscalar getMinRotation(fscalar angle) const {
            return angle - this->max_rotation_error;
        }
    };

    using FixedRotationModel = RotationModel<std::mt19937, detail::flat_distribution>;
    
    /*
     * MovementModel classes define how the robot's movement is affected by noise.
     */
    template <typename Trajectory>
    class MovementModel {
    public:
        Point_2 operator() (const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const;
        Trajectory generateTrajectory(const Point_2& origin, fscalar angle, const ConfigurationGeometry& configuration_environment) const;
    };
    
}
#endif
