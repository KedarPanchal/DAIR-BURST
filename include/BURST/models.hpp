#ifndef BURST_MODELS_HPP
#define BURST_MODELS_HPP

#include <CGAL/Polygon_2_algorithms.h>
#include <boost/multiprecision/mpfr.hpp>

#include <type_traits>
#include <random>
#include <optional>
#include <algorithm>
#include <iterator>
#include <vector>

#include "BURST/geometric_types.hpp"
#include "numeric_types.hpp"
#include "configuration_space.hpp"

// Contains models for how the robot's rotation and movement are affected by noise
namespace BURST::models {
    
    // -- ROTATION MODEL -------------------------------------------------------

    /*
     * RotationModel defines how the robot's rotation is affected by noise.
     */
    template <numeric::valid_rng PRNG = std::mt19937, numeric::valid_distribution<PRNG> Dist = std::uniform_real_distribution<double>>
    class RotationModel {
    private:
        numeric::fscalar max_rotation_error;
        mutable PRNG prng;
        mutable Dist rand_dist; // Generate from -1 to 1 to scale max_rotation_error by

    public:
        RotationModel(numeric::fscalar max_rotation_error, unsigned int seed = std::random_device{}()) : max_rotation_error{max_rotation_error}, prng{seed}, rand_dist{-1.0, 1.0} {}

        numeric::fscalar operator() (numeric::fscalar angle) const {
            // Generate a random rotation error scaled by max_rotation_error
            return angle + this->rand_dist(this->prng) * max_rotation_error;
        }

        numeric::fscalar max(numeric::fscalar angle) const {
            return angle + this->max_rotation_error;
        }
        numeric::fscalar min(numeric::fscalar angle) const {
            return angle - this->max_rotation_error;
        }
    };

    using MaximumRotationModel = RotationModel<std::mt19937, numeric::flat_distribution>;

    // Internal implementations not intended for public use
    namespace detail {
        /*
         * Declare type traits to check if a type is a valid rotation model
         * This means it must be an instantiation of the RotationModel template
         * Since the library is targeting C++20, this SFINAE needs to be used in tandem with concepts, since is_specialization_of is a C++23 feature
         */
        template <typename T>
        struct is_valid_rotation_model : std::false_type {};
        template <typename PRNG, typename Dist>
        struct is_valid_rotation_model<BURST::models::RotationModel<PRNG, Dist>> : std::true_type {};
    }
    
    // Checks if a type is a valid rotation model, as defined above, and wraps it as a concept
    template <typename R>
    concept valid_rotation_model = detail::is_valid_rotation_model<R>::value;

    
    // -- MOVEMENT MODEL -------------------------------------------------------

    /*
     * MovementModel defines how the robot's movement is affected by noise.
     */
    template <geometry::valid_trajectory_type Trajectory, geometry::valid_path_type Path>
    class MovementModel {
    public:
        std::optional<geometry::Point2D> operator() (const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // If the origin doesn't lie on the configuration space boundary, then the path is invalid, so return nullopt
            if (!configuration_space.intersection(origin).has_value()) return std::nullopt;

            // Create a direction vector from the angle
            numeric::hpscalar hp_angle = numeric::to_high_precision(angle);
            geometry::Vector2D direction_vector{boost::multiprecision::cos(hp_angle), boost::multiprecision::sin(hp_angle)};
            // Create a trajectory from the origin and direction vector
            Trajectory trajectory{origin, direction_vector};

            // Create a vector to store the intersection points since there can be multiple with a curvilinear polygon
            std::vector<geometry::Point2D> intersection_points;
            // Get the intersection of the trajectory with the configuration space boundary
            size_t intersection_count = configuration_space.intersection<Trajectory, Path>(trajectory, std::back_inserter(intersection_points));
            if (intersection_count == 0) return std::nullopt; // If there are no intersections, then the path is invalid, so return nullopt
            // The closest intersection to the point of origin is the endpoint
            // This should never throw since there's already a check for no intersections
            geometry::Point2D endpoint = *std::min_element(intersection_points.begin(), intersection_points.end(), [&origin](const geometry::Point2D& a, const geometry::Point2D& b) {
                return CGAL::squared_distance(a, origin) < CGAL::squared_distance(b, origin);
            });

            // Check if the trajectory points inward or outward from the configuration space
            // This can be done by computing the midpoint of the trajectory from the origin to the endpoint and checking if it lies inside the configuration space
            // If it does, then the trajectory points inward, and the path is valid, so return the endpoint, otherwise return nullopt
            geometry::Point2D midpoint = geometry::midpoint(origin, endpoint);
            return configuration_space.contains(midpoint) ? std::optional<geometry::Point2D>{endpoint} : std::nullopt;
        }
        std::optional<Path> path(const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // Identify the endpoint of the path by using the operator() function
            std::optional<geometry::Point2D> maybe_endpoint = (*this)(origin, angle, configuration_space);

            // If the endpoint doesn't exist, then the path is invalid, so return nullopt
            if (!maybe_endpoint.has_value()) return std::nullopt;
            // If the origin and endpoint are the same, then the path is invalid, so return nullopt
            if (*maybe_endpoint == origin) return std::nullopt;
            // Otherwise generate a path from the origin to the endpoint and return it
            return std::optional<Path>{Path{origin, *maybe_endpoint}};
        }
    };

    using LinearMovementModel = MovementModel<geometry::Ray2D, geometry::Segment2D>;
    
    // Internal implementations not intended for public use
    namespace detail {
        /*
         * Declare type traits to check if a type is a valid movement model
         * This means it must be an instantiation of the MovementModel template
         * Since the library is targeting C++20, this SFINAE needs to be used in tandem with concepts, since is_specialization_of is a C++23 feature
         */
        template <typename T>
        struct is_valid_movement_model : std::false_type {};
        template <typename Trajectory, typename Path>
        struct is_valid_movement_model<BURST::models::MovementModel<Trajectory, Path>> : std::true_type {}; 
    }
    
    template <geometry::valid_trajectory_type T, geometry::valid_path_type P>
    using valid_movement_model = detail::is_valid_movement_model<MovementModel<T, P>>;
    
}
#endif
