#ifndef MODELS_HPP
#define MODELS_HPP

#include <CGAL/Surface_sweep_2_algorithms.h>

#include <utility>
#include <random>
#include <optional>
#include <array>
#include <vector>

#include "types.hpp"
#include "functions.hpp"
#include "configuration_geometry.hpp"

namespace BURST::models {
    
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
        using Path = Segment2D<AlgebraicKernel>;
        using Trajectory = Ray2D<AlgebraicKernel>;
    };

    /*
     * MovementModel defines how the robot's movement is affected by noise.
     */
    template <typename ModelType>
    class MovementModel {
    public:
        std::optional<Point2D<AlgebraicKernel>> operator() (const Point2D<AlgebraicKernel>& origin, fscalar angle, const BURST::geometry::ConfigurationGeometry& configuration_environment) const noexcept {
            // If the origin doesn't lie on the configuration geometry boundary, then the movement is invalid, so return nullopt
            if (!curved_has_point(configuration_environment, origin)) return std::nullopt;

            // Create a direction trajectory from the angle
            hpscalar hp_angle = to_high_precision(angle);
            hpscalar direction_x = bmp::cos(hp_angle);
            hpscalar direction_y = bmp::sin(hp_angle);
            
            /*
             * RIP Direction Checking Code (2026 - 2026)
             * There used to be code here to check whether the direction pointed outside the polygon.
             * This wasn't needed in the first place as if it pointed outside the polygon, it would not intersect with any edges in the first place.
             * TODO: Polygons with holes might put holes (heh) in this logic, so figure this out later.
             */
            
            // Find the first intersection of the trajectory with the configuration geometry edges
            for (auto edge_it = configuration_environment.edge_begin(); edge_it != configuration_environment.edge_end(); ++edge_it) {
                // Skip if the origin is on the current edge, since that's where the robot is currently located
                if (curved_has_point(edge_it, origin)) continue;
                
                // Convert the origin to a rational type for intersection
                auto origin_interval_x = CGAL::to_interval(origin.x());
                auto origin_interval_y = CGAL::to_interval(origin.y());
                hpscalar rational_origin_x = to_high_precision(origin_interval_x.first + origin_interval_x.second) / 2;
                hpscalar rational_origin_y = to_high_precision(origin_interval_y.first + origin_interval_y.second) / 2;

                // Convert the trajectory to a curve for intersection
                ConicTraits traits;
                auto construct_curve_2 = traits.construct_curve_2_object();
                ConicTraits::Curve_2 trajectory_curve = construct_curve_2(0, 0, 0, rscalar{direction_y}, rscalar{-direction_x}, rscalar{rational_origin_x * direction_y - rational_origin_y * direction_x});
                // Create a collection of curves to intersect
                std::array<ConicTraits::Curve_2, 2> to_intersect = {edge_it->curve(), trajectory_curve};
                std::vector<Point2D<AlgebraicKernel>> intersection_points; // Place intersection points here
                                                                    
                // Compute the intersection
                CGAL::compute_intersection_points(to_intersect.begin(), to_intersect.end(), std::back_inserter(intersection_points));
                // Find the closest intersection point to the origin, which is the endpoint of the movement
                auto min = std::min_element(intersection_points.begin(), intersection_points.end(), [&origin](const Point2D<AlgebraicKernel>& a, const Point2D<AlgebraicKernel>& b) {
                    return CGAL::squared_distance(origin, a) < CGAL::squared_distance(origin, b);
                }); 

                // If there is an intersection point (the range isn't empty), return the closest intersection point as the endpoint of the movement
                if (min != intersection_points.end()) return *min;
            }

            // No intersections with any edge, so the movement is invalid, so return nullopt
            return std::nullopt;
        }
        std::optional<typename ModelType::Path> generatePath(const Point2D<AlgebraicKernel>& origin, fscalar angle, const BURST::geometry::ConfigurationGeometry& configuration_environment) const noexcept {
            // Identify the endpoint of the path by using the operator() function
            std::optional<Point2D<AlgebraicKernel>> maybe_endpoint = (*this)(origin, angle, configuration_environment);

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
