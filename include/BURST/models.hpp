#ifndef BURST_MODELS_HPP
#define BURST_MODELS_HPP

#include <CGAL/Polygon_2_algorithms.h>
#include <boost/multiprecision/mpfr.hpp>

#include <utility>
#include <random>
#include <optional>

#include "numeric_types.hpp"
#include "configuration_space.hpp"

namespace BURST::models {
    
    // Internal implementations not intended for public use
    namespace detail {
        // Type traits for validating whether a type can be used in a CGAL intersection computation against a polygon edge
        template <typename T, typename = void>
        struct is_valid_builtin_intersection_type : std::false_type {};
        template <typename T>
        struct is_valid_builtin_intersection_type<T, std::void_t<decltype(CGAL::intersection(std::declval<T>(), std::declval<geometry::Segment2D>()))>> : std::true_type {};
        
        // Type traits for validating whether a type can be a Path for a movement model
        // This requires a 2-argument constructor that accepts start and end geometry::Point2Ds (like geometry::Segment2D)
        template <typename T, typename = void>
        struct is_valid_path_type : std::false_type {};
        template <typename T>
        struct is_valid_path_type<T, std::void_t<decltype(T{std::declval<geometry::Point2D>(), std::declval<geometry::Point2D>()})>> : std::true_type {};

        // Type traits for validating whether a type can be a Trajectory for a movement model
        // This requires a 2-argument constructor that accepts an origin geometry::Point2D and a direction geometry::Vector2D (like geometry::Ray2D)
        template <typename T, typename = void>
        struct is_valid_trajectory_type : std::false_type {};
        template <typename T>
        struct is_valid_trajectory_type<T, std::void_t<decltype(T{std::declval<geometry::Point2D>(), std::declval<geometry::Vector2D>()})>> : std::true_type {};
    }
    
    // Custom random number distribution that generates the same number for every RNG, which is useful for testing
    // This allows for templating the rotation model and avoiding inheritance
    class flat_distribution {
    public:
        flat_distribution(double = 0.0, double = 0.0) {} // Dummy constructor to match the interface of std::uniform_real_distribution
        template <typename RNG> double operator() (RNG& rng) const {
            return 1.0;
        }
    };

    /*
     * RotationModel defines how the robot's rotation is affected by noise.
     */
    template <typename PRNG = std::mt19937, typename Dist = std::uniform_real_distribution<double>>
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

        numeric::fscalar getMaxRotation(numeric::fscalar angle) const {
            return angle + this->max_rotation_error;
        }
        numeric::fscalar getMinRotation(numeric::fscalar angle) const {
            return angle - this->max_rotation_error;
        }
    };

    using MaximumRotationModel = RotationModel<std::mt19937, flat_distribution>;
   
    // Define Path-Trajectory pairs for movement models
    struct LinearModel {
        using Path = geometry::Segment2D;
        using Trajectory = geometry::Ray2D;
    };

    /*
     * MovementModel defines how the robot's movement is affected by noise.
     */
    template <typename ModelType>
    class MovementModel {
    // Validate type traits
    static_assert(detail::is_valid_path_type<typename ModelType::Path>::value, "The ModelType's Path must have a 2-argument constructor that accepts start and end geometry::Point2Ds");
    static_assert(detail::is_valid_trajectory_type<typename ModelType::Trajectory>::value, "The ModelType's Trajectory must have a 2-argument constructor that accepts an origin geometry::Point2D and a direction geometry::Vector2D");
    public:
        std::optional<geometry::Point2D> operator() (const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // If the origin doesn't lie on the configuration space boundary, then the path is invalid, so return nullopt
            if (!configuration_space.intersection(origin).has_value()) return std::nullopt;

            // Create a direction vector from the angle
            numeric::hpscalar hp_angle = numeric::to_high_precision(angle);
            geometry::Vector2D direction_vector{boost::multiprecision::cos(hp_angle), boost::multiprecision::sin(hp_angle)};
            // Create a trajectory from the origin and direction vector
            typename ModelType::Trajectory trajectory{origin, direction_vector};

            // Get the intersection of the trajectory with the configuration space boundary, which is the endpoint of the path
            return configuration_space.intersection(trajectory);
        }
        std::optional<typename ModelType::Path> generatePath(const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // Identify the endpoint of the path by using the operator() function
            std::optional<geometry::Point2D> maybe_endpoint = (*this)(origin, angle, configuration_space);

            // If the endpoint doesn't exist, then the path is invalid, so return nullopt
            if (!maybe_endpoint.has_value()) return std::nullopt;
            // If the origin and endpoint are the same, then the path is invalid, so return nullopt
            if (*maybe_endpoint == origin) return std::nullopt;
            // Otherwise generate a path from the origin to the endpoint and return it
            return std::optional<typename ModelType::Path>{typename ModelType::Path{origin, *maybe_endpoint}};
        }
    };

    using LinearMovementModel = MovementModel<LinearModel>;
    
}
#endif
