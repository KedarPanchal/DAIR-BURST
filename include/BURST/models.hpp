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

#include "geometry.hpp"
#include "numeric.hpp"
#include "configuration_space.hpp"
#include "logging.hpp"

/**
 * @file models.hpp
 * @brief Pluggable noise models for orientation and translation against a @ref geometry::ConfigurationSpace.
 *
 * These templates decouple how the robot's commanded heading is perturbed and how a straight or
 * curved trajectory is resolved to a boundary intersection from the @ref Robot geometry.
 */

namespace BURST::models {
    
    /**
     * @brief Bounded random perturbation applied to commanded orientations.
     *
     * Each sampled rotation error is drawn from `Dist` over `[-1, 1]` (by default) and scaled by
     * the absolute `max_rotation_error` bound. The model is stateful via the PRNG and is
     * copy-friendly for use inside @ref Robot.
     *
     * @tparam PRNG Engine satisfying @ref numeric::valid_rng (default `std::mt19937`).
     * @tparam Dist Distribution satisfying @ref numeric::valid_distribution with `PRNG`.
     */
    template <numeric::valid_rng PRNG = std::mt19937, numeric::valid_distribution<PRNG> Dist = std::uniform_real_distribution<double>>
    class RotationModel {
    private:
        numeric::fscalar max_rotation_error;
        mutable PRNG prng;
        mutable Dist rand_dist;

    public:
        /**
         * @param max_rotation_error Absolute bound on additive angle error (magnitude is taken).
         * @param seed PRNG seed; defaults to a non-deterministic source when available.
         */
        RotationModel(numeric::fscalar max_rotation_error, unsigned int seed = std::random_device{}()) : max_rotation_error{CGAL::abs(max_rotation_error)}, prng{seed}, rand_dist{-1.0, 1.0} {}

        /**
         * @brief Sample a perturbed angle: `angle + noise * max_rotation_error`.
         * @return Perturbed angle sample.
         */
        numeric::fscalar operator()(numeric::fscalar angle) const {
            // Generate a random rotation error scaled by max_rotation_error
            return angle + this->rand_dist(this->prng) * max_rotation_error;
        }

        /** 
         * @brief Upper limit of possible angles: `angle + max_rotation_error`.
         * @return Upper bound value.
         */
        numeric::fscalar max(numeric::fscalar angle) const {
            return angle + this->max_rotation_error;
        }
        /** 
         * @brief Lower limit of possible angles: `angle - max_rotation_error`.
         * @return Lower bound value.
         */
        numeric::fscalar min(numeric::fscalar angle) const {
            return angle - this->max_rotation_error;
        }
    };

    /** @brief @ref RotationModel with deterministic noise (see @ref numeric::flat_distribution). */
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
    
    /** @brief True if `R` is some `RotationModel<PRNG, Dist>` specialization. */
    template <typename R>
    concept valid_rotation_model = detail::is_valid_rotation_model<R>::value;

    
    /**
     * @brief Maps a motion command to a feasible boundary point on the configuration space.
     *
     * Given a starting point on the boundary of `configuration_space` and a heading `angle`,
     * constructs a `Trajectory` ray from the origin along the corresponding unit vector, finds
     * intersections with the boundary, and selects the endpoint that lies along the inward
     * direction (validated by the midpoint test). If the origin is not on the boundary, no
     * intersection exists, or the motion would leave the configuration space, returns `std::nullopt`.
     *
     * @tparam Trajectory Trajectory type satisfying @ref geometry::valid_trajectory_type.
     * @tparam Path       Path type satisfying @ref geometry::valid_path_type for the chord joining
     *                    start and end boundary points.
     */
    template <geometry::valid_trajectory_type Trajectory, geometry::valid_path_type Path>
    class MovementModel {
    public:
        /**
         * @brief Resolve the endpoint of a valid inward motion, if one exists.
         * @param origin Point on the configuration-space boundary (see @ref geometry::ConfigurationSpace::onEdge).
         * @param angle Heading in radians defining the motion direction.
         * @param configuration_space Configuration space for the robot.
         * @return Endpoint on the boundary if the motion is valid, `std::nullopt` otherwise.
         */
        std::optional<geometry::Point2D> operator()(const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // If the origin doesn't lie on the configuration space boundary, then the path is invalid, so return nullopt
            if (!configuration_space.onEdge(origin)) {
                BURST_ERROR("Origin point does not lie on the configuration space boundary, path is invalid");
                return std::nullopt;
            }

            // Create a direction vector from the angle
            numeric::hpscalar hp_angle = numeric::to_high_precision(angle);
            geometry::Vector2D direction_vector{boost::multiprecision::cos(hp_angle), boost::multiprecision::sin(hp_angle)};
            // Create a trajectory from the origin and direction vector
            Trajectory trajectory{origin, direction_vector};

            // Create a vector to store the intersection points since there can be multiple with a curvilinear polygon
            std::vector<geometry::Point2D> intersection_points;
            // Get the intersection of the trajectory with the configuration space boundary
            size_t intersection_count = configuration_space.intersection<Trajectory, Path>(trajectory, std::back_inserter(intersection_points));
            // If there are no intersections, then the path is invalid, so return nullopt
            if (intersection_count == 0) {
                BURST_ERROR("Trajectory does not intersect with the configuration space boundary, path is invalid");
                return std::nullopt;
            }
            // The closest intersection to the point of origin is the endpoint
            // This should never throw since there's already a check for no intersections
            geometry::Point2D endpoint = *std::min_element(intersection_points.begin(), intersection_points.end(), [&origin](const geometry::Point2D& a, const geometry::Point2D& b) {
                return CGAL::squared_distance(a, origin) < CGAL::squared_distance(b, origin);
            });

            // Check if the trajectory points inward or outward from the configuration space
            // This can be done by computing the midpoint of the trajectory from the origin to the endpoint and checking if it lies inside the configuration space
            // If it does, then the trajectory points inward, and the path is valid, so return the endpoint, otherwise return nullopt
            geometry::Point2D midpoint = geometry::midpoint(origin, endpoint);
            if (configuration_space.contains(midpoint)) {
                return endpoint;
            } else {
                BURST_ERROR("Trajectory points outward from the configuration space, path is invalid");
                return std::nullopt;
            }
        }
        /**
         * @brief Same as @ref operator() but returns a `Path` segment (or curve) from `origin` to the endpoint.
         *
         * The path is empty if the motion is invalid or degenerate (same start and end).
         *
         * @return Path from `origin` to endpoint if valid and non-degenerate, `std::nullopt` otherwise.
         */
        std::optional<Path> path(const geometry::Point2D& origin, numeric::fscalar angle, const BURST::geometry::ConfigurationSpace& configuration_space) const noexcept {
            // Identify the endpoint of the path by using the operator() function
            std::optional<geometry::Point2D> maybe_endpoint = (*this)(origin, angle, configuration_space);

            // If the endpoint doesn't exist, then the path is invalid, so return nullopt
            if (!maybe_endpoint.has_value()) return std::nullopt;
            // If the origin and endpoint are the same, then the path is invalid, so return nullopt
            if (*maybe_endpoint == origin) {
                BURST_ERROR("Origin and endpoint of the path are the same, path is invalid");
                return std::nullopt;
            }
            // Otherwise generate a path from the origin to the endpoint and return it
            return Path{origin, *maybe_endpoint};
        }
    };

    /** @brief Standard motion model: infinite ray trajectory clipped to a straight segment path. */
    using LinearMovementModel = MovementModel<geometry::Ray2D, geometry::Segment2D>;
    
    namespace detail {
        template <typename T>
        struct is_valid_movement_model : std::false_type {};
        template <typename Trajectory, typename Path>
        struct is_valid_movement_model<BURST::models::MovementModel<Trajectory, Path>> : std::true_type {}; 
    }
    
    /** @brief True if `M` is some `MovementModel<Trajectory, Path>` specialization. */
    template <typename M>
    concept valid_movement_model = detail::is_valid_movement_model<M>::value;
    
}
#endif
