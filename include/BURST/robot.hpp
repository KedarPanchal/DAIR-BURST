#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <memory>
#include <random>
#include <array>
#include <numeric>
#include <unordered_map>
#include <algorithm>

#include <boost/multiprecision/mpfr.hpp>

#include "numeric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "models.hpp"
#include "logging.hpp"

namespace BURST {

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    template <
        geometry::valid_trajectory_type Trajectory, 
        geometry::valid_path_type Path,
        numeric::valid_rng PRNG = std::mt19937, 
        numeric::valid_distribution<PRNG> Dist = std::uniform_real_distribution<double>
    >
    class Robot : public Renderable {
    private:
        numeric::fscalar radius;
        geometry::Point2D position;
        std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_environment;

        models::RotationModel<PRNG, Dist> rotation_model;
        models::MovementModel<Trajectory, Path> movement_model;

        // Custom hash function for points used as a helper
        // Quick and dirty so we don't need to be overly concerned with collisions
        struct PointHash {
            std::size_t operator() (const geometry::Point2D& point) const {
                // Scale the x and y coordinates to integers
                size_t hx = CGAL::to_double(point.x()) * 1e6;
                size_t hy = CGAL::to_double(point.y()) * 1e6;
                // Use Cantor pairing to combine two integers into a single hash value
                return (hx + hy) * (hx + hy + 1) / 2;
            }
        };

    protected:
        // Protected constructor since preconditions are validated by public static create functions
        Robot(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<PRNG, Dist> rotation_model, models::MovementModel<Trajectory, Path> movement_model) : 
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{rotation_model}, 
            movement_model{movement_model} {}
       
    public:
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) return std::nullopt;
            else return std::optional<Robot>{Robot{robot_radius, starting_point, models::MaximumRotationModel{max_rotation_error}, models::MovementModel<Trajectory, Path>{}}};
        }
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error, unsigned int rotation_seed) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) return std::nullopt;
            else return std::optional<Robot>{Robot{robot_radius, starting_point, models::MaximumRotationModel{max_rotation_error, rotation_seed}, models::MovementModel<Trajectory, Path>{}}};
        }
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<PRNG, Dist> rotation_model, models::MovementModel<Trajectory, Path> movement_model) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) return std::nullopt;
            else return std::optional<Robot>{Robot{robot_radius, starting_point, rotation_model, movement_model}};
        }

        // Precondition: The robot is on the border of the configuration space
        void setConfigurationEnvironment(std::unique_ptr<BURST::geometry::ConfigurationSpace> config_environment) {
            this->configuration_environment = std::move(config_environment);
            if (!this->configuration_environment->intersection(this->position).has_value()) {
                BURST_WARNING("Robot's current position (" + this->position.x() + ", " + this->position.y() + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.");
            }
        }
        void setPosition(const geometry::Point2D& new_position) {
            this->position = new_position;
            if (!this->configuration_environment->intersection(this->position).has_value()) {
                BURST_WARNING("Robot's new position (" + this->position.x() + ", " + this->position.y() + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.");
            }
        }
        const BURST::geometry::ConfigurationSpace& getConfigurationEnvironment() const {
            return *this->configuration_environment;
        }
        numeric::fscalar getRadius() const {
            return this->radius;
        }

        numeric::fscalar perturb(numeric::fscalar angle) const {
            return this->rotation_model(angle);
        }

        std::optional<geometry::Point2D> shootRay(numeric::fscalar angle, bool perturbed = false) const {
            auto trajectory = this->movement_model.path(this->position, perturbed ? this->rotation_model(angle) : angle, *this->configuration_environment);
            return trajectory.has_value() ? std::optional<geometry::Point2D>{trajectory->endpoint()} : std::nullopt;
        }
        std::optional<geometry::CurvilinearPolygonSet2D> coveredArea(numeric::fscalar angle, bool perturbed = false) const {
            numeric::fscalar effective_angle = perturbed ? this->rotation_model(angle) : angle;
            // Generate an endpoint for the robot's movement trajectory
            std::optional<geometry::Point2D> endpoint = this->movement_model(this->position, effective_angle, *this->configuration_environment);
            // If the trajectory is nullopt, we can't generate a stadium, so return nullopt
            if (!endpoint.has_value()) return std::nullopt;

            // Add the robot's start and end circles to the stadium polygon set
            geometry::CurvilinearPolygonSet2D stadium;
            // Insert the circle for the robot's starting position
            stadium.insert(*geometry::construct_circle(this->radius, this->position));
            // Insert the circle for the robot's ending position
            stadium.insert(*geometry::construct_circle(this->radius, *endpoint));

            // Create the somewhat-rectangular portion of the stadium, with the edges defined by the robot's path type
            // The angle perpendicular to the movement direction is the angle between the movement vector of the robot and its diameter containing the rectangle vertices
            numeric::hpscalar diameter_angle = numeric::to_high_precision(effective_angle + CGAL_PI / 2);
            // Compute the difference between the center of the robot and the rectangle vertices in the x and y directions of the diameter perpendicular to the robot's movement direction
            numeric::fscalar dx = this->radius * numeric::to_fscalar(boost::multiprecision::cos(diameter_angle));
            numeric::fscalar dy = this->radius * numeric::to_fscalar(boost::multiprecision::sin(diameter_angle));
            // Compute the vertices of the rectangle by adding and subtracting dx and dy from the start and end points of the robot's trajectory
            std::array<geometry::Point2D, 4> rectangle_vertices{
                geometry::Point2D{this->position.x() + dx, this->position.y() + dy},
                geometry::Point2D{this->position.x() - dx, this->position.y() - dy},
                geometry::Point2D{endpoint->x() - dx, endpoint->y() - dy},
                geometry::Point2D{endpoint->x() + dx, endpoint->y() + dy}
            };
            
            // Sort the rectangle vertices in counterclockwise order to ensure the correct orientation for CGAL
            // This can be done by computing the average of the vertices and sorting based on the angle from the average to each vertex
            geometry::Point2D average = *geometry::average(rectangle_vertices);
            // Then store the atan from each point to the average in an unordered map to avoid recomputing it
            std::unordered_map<geometry::Point2D, numeric::fscalar, PointHash> angle_map;
            for (const geometry::Point2D& vertex : rectangle_vertices) {
                numeric::hpscalar opposite = numeric::to_high_precision(vertex.y() - average.y());
                numeric::hpscalar adjacent = numeric::to_high_precision(vertex.x() - average.x());
                angle_map[vertex] = numeric::to_fscalar(boost::multiprecision::atan2(opposite, adjacent));
            }
            // Sort the vertices based on their angle to the average point in counterclockwise order
            std::sort(rectangle_vertices.begin(), rectangle_vertices.end(), [&average, &angle_map](const geometry::Point2D& a, const geometry::Point2D& b) {
                return angle_map[a] < angle_map[b];
            });

            // Using the sorted rectangle vertices, construct them pairwise into diameter segments and paths in CCW order
            boost::container::small_vector<CurvedTraits::X_monotone_curve_2, 4> rectangle_edges;
            for (size_t i = 0; i < rectangle_vertices.size() - 1; ++i) {
                // Check if both points are on a diameter (i.e., their midpoint is the start or end point of the robot's trajectory)
                // If so, construct a diameter segment, otherwise construct a Path
                geometry::Point2D midpoint = geometry::midpoint(rectangle_vertices[i], rectangle_vertices[i + 1]);
                if (midpoint == this->position || midpoint == *endpoint) {
                    rectangle_edges.emplace_back(geometry::construct_curve(geometry::Segment2D{rectangle_vertices[i], rectangle_vertices[i + 1]}));
                } else {
                    rectangle_edges.emplace_back(geometry::construct_curve(Path{rectangle_vertices[i], rectangle_vertices[i + 1]}));
                }
            }
            // Form a polygon from the rectangle edges and insert it into the stadium
            stadium.insert(geometry::CurvilinearPolygon2D{rectangle_edges.begin(), rectangle_edges.end()});
            return std::optional<geometry::CurvilinearPolygonSet2D>{stadium};
        }
        geometry::Polygon2D certainlyCoveredArea(numeric::fscalar angle) const;
        void move(numeric::fscalar angle, bool perturbed = false) {
            numeric::fscalar effective_angle = perturbed ? this->rotation_model(angle) : angle;
            // Generate an endpoint for the robot's movement trajectory
            std::optional<geometry::Point2D> endpoint = this->movement_model(this->position, effective_angle, *this->configuration_environment);
            // If the trajectory is nullopt, we can't move the robot, so return without changing the robot's position
            if (!endpoint.has_value()) return;
            // Otherwise, move the robot to the endpoint
            this->position = *endpoint;
        }

        void render(graphics::Scene& scene) const override;
    };

}

#endif
