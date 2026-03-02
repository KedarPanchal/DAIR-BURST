#ifndef BURST_TRAITS_HPP
#define BURST_TRAITS_HPP

#include "geometric_types.hpp"

namespace BURST::models {

    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    };

    template <typename T>
    concept valid_trajectory_type = requires (geometry::Point2D origin, geometry::Vector2D direction) {
        T{origin, direction};
    };

    // Custom random number distribution that generates the same number for every RNG, which is useful for testing
    // This allows for templating the rotation model and avoiding inheritance
    class flat_distribution {
    public:
        flat_distribution(double = 0.0, double = 0.0) {} // Dummy constructor to match the interface of std::uniform_real_distribution
        template <typename RNG> double operator() (RNG& rng) const {
            return 1.0;
        }
    };

}

#endif
