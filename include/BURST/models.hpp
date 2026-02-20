#ifndef MODELS_HPP
#define MODELS_HPP

#include <CGAL/Polygon_2_algorithms.h>

#include <utility>
#include <random>
#include <optional>

#include "types.hpp"
#include "functions.hpp"
#include "configuration_geometry.hpp"

namespace BURST::models {
    
    // Internal implementations not intended for public use
    namespace detail {
        // Type traits for validating whether a type can be used in a CGAL intersection computation against a polygon edge
        template <typename T, typename = void>
        struct is_valid_builtin_intersection_type : std::false_type {};
        template <typename T>
        struct is_valid_builtin_intersection_type<T, std::void_t<decltype(CGAL::intersection(std::declval<T>(), std::declval<AlgebraicSegment2D>()))>> : std::true_type {};
        
        // Type traits for validating whether a type can be a Path for a movement model
        // This requires a 2-argument constructor that accepts start and end Point2Ds (like Segment2D)
        template <typename T, typename = void>
        struct is_valid_path_type : std::false_type {};
        template <typename T>
        struct is_valid_path_type<T, std::void_t<decltype(T{std::declval<AlgebraicPoint2D>(), std::declval<AlgebraicPoint2D>()})>> : std::true_type {};

        // Type traits for validating whether a type can be a Trajectory for a movement model
        // This requires a 2-argument constructor that accepts an origin Point2D and a direction Vector2D (like Ray2D)
        template <typename T, typename = void>
        struct is_valid_trajectory_type : std::false_type {};
        template <typename T>
        struct is_valid_trajectory_type<T, std::void_t<decltype(T{std::declval<AlgebraicPoint2D>(), std::declval<Vector2D>()})>> : std::true_type {};
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

    using MaximumRotationModel = RotationModel<std::mt19937, flat_distribution>;
   
    // Define Path-Trajectory pairs for movement models
    struct LinearModel {
        using Path = AlgebraicSegment2D;
        using Trajectory = AlgebraicRay2D;
    };

    /*
     * MovementModel defines how the robot's movement is affected by noise.
     */
    template <typename ModelType>
    class MovementModel {
    // Validate type traits
    static_assert(detail::is_valid_path_type<typename ModelType::Path>::value, "The ModelType's Path must have a 2-argument constructor that accepts start and end Point2Ds");
    static_assert(detail::is_valid_trajectory_type<typename ModelType::Trajectory>::value, "The ModelType's Trajectory must have a 2-argument constructor that accepts an origin Point2D and a direction Vector2D");
    public:
        std::optional<AlgebraicPoint2D> operator() (const AlgebraicPoint2D& origin, fscalar angle, const BURST::geometry::ConfigurationGeometry& configuration_environment) const noexcept {
            // If the origin doesn't lie on the configuration geometry boundary, then the movement is invalid, so return nullopt
            if (!curved_has_point(configuration_environment, origin)) return std::nullopt;

            // Create a direction trajectory from the angle
            hpscalar hp_angle = to_high_precision(angle);
            Vector2D direction_vector{bmp::cos(hp_angle), bmp::sin(hp_angle)};
            
            /*
             * RIP Direction Checking Code (2026 - 2026)
             * There used to be code here to check whether the direction pointed outside the polygon.
             * This wasn't needed in the first place as if it pointed outside the polygon, it would not intersect with any edges in the first place.
             * TODO: Polygons with holes might put holes (heh) in this logic, so figure this out later.
             */
            
            // Create a trajectory from the origin in the direction of the direction vector
            typename ModelType::Trajectory trajectory{origin, direction_vector};

            // Find the first intersection of the trajectory with the configuration geometry edges
            for (auto edge_it = configuration_environment.edge_begin(); edge_it != configuration_environment.edge_end(); ++edge_it) {
                // Skip if the origin is on the current edge, since that's where the robot is currently located
                if (curved_has_point(edge_it, origin)) continue;
                
                // Use CGAL's inbuilt intersection function if Trajectory is a valid type for intersection with the polygon edge
                if constexpr (detail::is_valid_builtin_intersection_type<typename ModelType::Trajectory>::value) {
                    auto maybe_intersection = CGAL::intersection(trajectory, *edge_it);

                    // No intersection, so continue to the next edge
                    if (!maybe_intersection.has_value()) continue;

                    // If the intersection is a point we found the next position of the robot, so return it
                    if (const AlgebraicPoint2D* intersection_point = std::get_if<AlgebraicPoint2D>(&*maybe_intersection)) return std::optional<AlgebraicPoint2D>{*intersection_point};

                    /*
                     * The case of the trajectory being collinear with the edge shouldn't occur since the edge intersecting with the origin isn't considered
                     * For the sake of completeness, the following is considered:
                     * If the trajectory is collinear with the edge, the robot should intersect with another edge in the polygon given:
                     * 1. The intersecting edge forms a convex vertex with another edge, so the robot can intersect the next edge at that vertex
                     * 2. The intersecting edge forms a concave vertex with another edge, so the robot slides along the edge's direction until it intersects with another edge
                     * Because of this, we can just continue to the next edge
                     */
                    // Continue to the next edge...
                } else {
                    // TODO: Figure out some interface for handling the case where the trajectory type isn't directly compatible with CGAL's intersection function
                }
            }

            // No intersections with any edge, so the movement is invalid, so return nullopt
            return std::nullopt;
        }
        std::optional<typename ModelType::Path> generatePath(const AlgebraicPoint2D& origin, fscalar angle, const BURST::geometry::ConfigurationGeometry& configuration_environment) const noexcept {
            // Identify the endpoint of the path by using the operator() function
            std::optional<AlgebraicPoint2D> maybe_endpoint = (*this)(origin, angle, configuration_environment);

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
