#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <memory>
#include <random>
#include <array>
#include <unordered_map>
#include <algorithm>

#include <boost/multiprecision/mpfr.hpp>
#include <boost/functional/hash.hpp>

#include "geometric_types.hpp"
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
        geometry::valid_trajectory_type T = geometry::Ray2D, 
        geometry::valid_path_type P = geometry::Segment2D,
        numeric::valid_rng R = std::mt19937, 
        numeric::valid_distribution<R> D = std::uniform_real_distribution<double>
    >
    class Robot : public Renderable {
    private:
        numeric::fscalar radius;
        geometry::Point2D position;
        std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_environment;

        models::RotationModel<R, D> rotation_model;
        models::MovementModel<T, P> movement_model;

        // Custom hash function for points used as a helper
        // Quick and dirty so we don't need to be overly concerned with collisions
        struct PointHash {
            std::size_t operator() (const geometry::Point2D& point) const {
                size_t seed = 0; 
                boost::hash_combine(seed, CGAL::to_double(point.x()));
                boost::hash_combine(seed, CGAL::to_double(point.y()));
                return seed;
            }
        };

    protected:
        // Protected constructor since preconditions are validated by public static create functions
        Robot(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<R, D> rotation_model, models::MovementModel<T, P> movement_model) : 
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{rotation_model}, 
            movement_model{movement_model} {}
              
    public:
        using Trajectory = T;
        using Path = P;
        using PRNG = R;
        using Dist = D;
        using RotationModelType = models::RotationModel<R, D>;
        using MovementModelType = models::MovementModel<T, P>;

        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                BURST_ERROR("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return std::optional<Robot>{Robot{robot_radius, starting_point, models::RotationModel<R, D>{max_rotation_error}, models::MovementModel<T, P>{}}};
        }
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error, unsigned int rotation_seed) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                BURST_ERROR("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return std::optional<Robot>{Robot{robot_radius, starting_point, models::RotationModel<R, D>{max_rotation_error, rotation_seed}, models::MovementModel<T, P>{}}};
        }
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<R, D> rotation_model, models::MovementModel<T, P> movement_model) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                BURST_ERROR("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return std::optional<Robot>{Robot{robot_radius, starting_point, rotation_model, movement_model}};
        }
        const BURST::geometry::ConfigurationSpace& getConfigurationEnvironment() const {
            return *this->configuration_environment;
        }
        numeric::fscalar getRadius() const {
            return this->radius;
        }
        BURST::geometry::Point2D getPosition() const {
            return this->position;
        }
        // Precondition: The robot is on the border of the configuration space
        void setConfigurationEnvironment(std::unique_ptr<BURST::geometry::ConfigurationSpace> config_environment) {
            this->configuration_environment = std::move(config_environment);
            if (!this->configuration_environment->intersection(this->position)) {
                std::string warning_string = "Robot's current position (" + BURST::numeric::to_string(this->position.x()) + ", " + BURST::numeric::to_string(this->position.y()) + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.";
                BURST_WARNING(warning_string.c_str());
            }
        }
        void setPosition(const geometry::Point2D& new_position) {
            this->position = new_position;
            if (!this->configuration_environment->intersection(this->position)) {
                std::string warning_string = "Robot's new position (" + BURST::numeric::to_string(this->position.x()) + ", " + BURST::numeric::to_string(this->position.y()) + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.";
                BURST_WARNING(warning_string.c_str());
            }
        }

        numeric::fscalar perturb(const numeric::fscalar& angle) const {
            return this->rotation_model(angle);
        }

        std::optional<geometry::Point2D> shootRay(const numeric::fscalar& angle, bool perturbed = false) const {
            // Cannot shoot ray if configuration environment does not exist
            if (!this->configuration_environment) return std::nullopt;
            return this->movement_model(this->position, perturbed ? this->rotation_model(angle) : angle, *this->configuration_environment);
        }
        std::optional<geometry::CurvilinearPolygonSet2D> coveredArea(const numeric::fscalar& angle, bool perturbed = false) const {
            // Cannot generate a stadium if configuration environment does not exist
            if (!this->configuration_environment) return std::nullopt;

            numeric::fscalar effective_angle = perturbed ? this->rotation_model(angle) : angle;
            // Generate an endpoint for the robot's movement trajectory
            std::optional<geometry::Point2D> endpoint = this->movement_model(this->position, effective_angle, *this->configuration_environment);
            // If the trajectory is nullopt, we can't generate a stadium, so return nullopt
            if (!endpoint.has_value()) return std::nullopt;

            // Add the robot's start and end circles to the stadium polygon set
            geometry::CurvilinearPolygonSet2D stadium;
            // Add the circle for the robot's starting position
            stadium.join(*geometry::construct_circle(this->radius, this->position));
            // Add the circle for the robot's ending position
            stadium.join(*geometry::construct_circle(this->radius, *endpoint));

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
                return angle_map.at(a) < angle_map.at(b);
            });

            // Using the sorted rectangle vertices, construct them pairwise into diameter segments and paths in CCW order
            boost::container::small_vector<CurvedTraits::X_monotone_curve_2, 4> rectangle_edges;
            for (size_t i = 0; i < rectangle_vertices.size(); ++i) {
                size_t next = (i + 1) % rectangle_vertices.size();
                // Check if both points are on a diameter (i.e., their midpoint is the start or end point of the robot's trajectory)
                // If so, construct a diameter segment, otherwise construct a P
                geometry::Point2D midpoint = geometry::midpoint(rectangle_vertices[i], rectangle_vertices[next]);
                if (midpoint == this->position || midpoint == *endpoint) rectangle_edges.emplace_back(geometry::construct_curve(geometry::Segment2D{rectangle_vertices[i], rectangle_vertices[next]}));
                else rectangle_edges.emplace_back(geometry::construct_curve(P{rectangle_vertices[i], rectangle_vertices[next]}));
            }
            // Form a polygon from the rectangle edges and add it into the stadium
            stadium.join(geometry::CurvilinearPolygon2D{rectangle_edges.begin(), rectangle_edges.end()});
            return std::optional<geometry::CurvilinearPolygonSet2D>{stadium};
        }

        bool move(const numeric::fscalar& angle, bool perturbed = false) {
            // Cannot move if configuration environment does not exist
            if (!this->configuration_environment) return false;

            numeric::fscalar effective_angle = perturbed ? this->rotation_model(angle) : angle;
            // Generate an endpoint for the robot's movement trajectory
            std::optional<geometry::Point2D> endpoint = this->movement_model(this->position, effective_angle, *this->configuration_environment);
            // If the trajectory is nullopt, we can't move the robot, so return without changing the robot's position
            if (!endpoint.has_value()) return false;
            // Otherwise, move the robot to the endpoint
            this->position = *endpoint;
            return true;
        }
        
        // TODO: Implement render function once the graphics library bugs are squashed
        void render(graphics::Scene& scene) const override {
        }
    };

}

#endif
