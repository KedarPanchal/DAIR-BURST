#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <memory>
#include <random>
#include <array>
#include <unordered_map>
#include <algorithm>

#include <boost/multiprecision/mpfr.hpp>
#include <boost/functional/hash.hpp>
#include <boost/container/small_vector.hpp>

#include "geometry.hpp"
#include "numeric.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "models.hpp"
#include "logging.hpp"

/**
 * @file robot.hpp
 * @brief Circular robot with configurable rotation and movement noise models in a configuration space.
 */

namespace BURST {

    /**
     * @brief Kinematic agent modeled as a disk with stochastic heading error and boundary-constrained motion.
     *
     * The robot carries a @ref models::RotationModel for perturbed turns and a
     * @ref models::MovementModel that resolves straight or curved moves against a shared
     * @ref geometry::ConfigurationSpace. It is drawable as a filled disk via @ref renderable::Renderable.
     *
     * @tparam T Trajectory type for the movement model (default @ref geometry::Ray2D).
     * @tparam P Path type for boundary-to-boundary segments (default @ref geometry::Segment2D).
     * @tparam R PRNG type for the rotation model (default `std::mt19937`).
     * @tparam D Distribution type for the rotation model (default `std::uniform_real_distribution<double>`).
     */
    template <
        geometry::valid_trajectory_type T = geometry::Ray2D, 
        geometry::valid_path_type P = geometry::Segment2D,
        numeric::valid_rng R = std::mt19937, 
        numeric::valid_distribution<R> D = std::uniform_real_distribution<double>
    >
    class Robot : public renderable::Renderable {
    private:
        numeric::fscalar radius;
        geometry::Point2D position;
        std::shared_ptr<BURST::geometry::ConfigurationSpace> configuration_environment;

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
            Renderable{},
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{rotation_model}, 
            movement_model{movement_model} {}

    public:
        using Trajectory = T;           /**< Trajectory template parameter. */
        using Path = P;                 /**< Path template parameter. */
        using PRNG = R;                 /**< Rotation PRNG type. */
        using Dist = D;                 /**< Rotation distribution type. */
        using RotationModelType = models::RotationModel<R, D>;   /**< Concrete rotation model type. */
        using MovementModelType = models::MovementModel<T, P>;     /**< Concrete movement model type. */

        /**
         * @brief Construct a robot with default-constructed models and a rotation bound.
         * @param robot_radius Physical radius of the disk; must be positive.
         * @param starting_point Initial center position.
         * @param max_rotation_error Absolute bound passed to @ref models::RotationModel.
         * @return `std::nullopt` if `robot_radius <= 0`.
         */
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                burst_error("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return Robot{robot_radius, starting_point, models::RotationModel<R, D>{max_rotation_error}, models::MovementModel<T, P>{}};
        }
        /**
         * @brief Same as @ref create with explicit PRNG seed for reproducible rotation noise.
         * @return `std::nullopt` if `robot_radius <= 0`.
         */
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error, unsigned int rotation_seed) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                burst_error("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return Robot{robot_radius, starting_point, models::RotationModel<R, D>{max_rotation_error, rotation_seed}, models::MovementModel<T, P>{}};
        }
        /**
         * @brief Construct a robot with fully custom rotation and movement models.
         * @return `std::nullopt` if `robot_radius <= 0`.
         */
        static std::optional<Robot> create(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<R, D> rotation_model, models::MovementModel<T, P> movement_model) {
            // Cannot construct a robot with a non-positive radius
            if (robot_radius <= 0) {
                burst_error("Cannot construct a robot with non-positive radius");
                return std::nullopt;
            }
            else return Robot{robot_radius, starting_point, rotation_model, movement_model};
        }
        /** @brief Read-only view of the current configuration space (undefined if never set). */
        const BURST::geometry::ConfigurationSpace& getConfigurationEnvironment() const {
            return *this->configuration_environment;
        }
        /** 
         * @brief Disk radius in workspace units.
         * @return Robot radius.
         */
        numeric::fscalar getRadius() const {
            return this->radius;
        }
        /** 
         * @brief Current center of the robot.
         * @return Current robot center position.
         */
        BURST::geometry::Point2D getPosition() const {
            return this->position;
        }
        /**
         * @brief Attach the configuration space used for motion and coverage queries.
         *
         * If the robot's current position is not on the boundary of the new space, a warning may
         * be logged because subsequent moves assume a boundary start configuration.
         */
        void setConfigurationEnvironment(std::shared_ptr<BURST::geometry::ConfigurationSpace> config_environment) {
            this->configuration_environment = std::move(config_environment);
            if (!this->configuration_environment->onEdge(this->position)) {
                std::string warning_string = "Robot's current position (" + BURST::numeric::to_string(this->position.x()) + ", " + BURST::numeric::to_string(this->position.y()) + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.";
                burst_warning(warning_string.c_str());
            }
        }
        /**
         * @brief Get the configuration space.
         * @return Reference to the attached configuration space.
         */
        const geometry::ConfigurationSpace& getConfigurationEnvironment() {
            return *this->configuration_environment;
        }
        /** 
         * @brief Get the configuration space as a shared pointer.
         * @return Shared pointer to the attached configuration space.
         */
        std::shared_ptr<geometry::ConfigurationSpace> getConfigurationEnvironmentPtr() {
            return this->configuration_environment;
        }
        /**
         * @brief Move the robot's center to `new_position`.
         *
         * If a configuration environment is set, may warn when the position is not consistent with
         * expected boundary motion.
         */
        void setPosition(const geometry::Point2D& new_position) {
            this->position = new_position;
            if (!this->configuration_environment->intersection(this->position)) {
                std::string warning_string = "Robot's new position (" + BURST::numeric::to_string(this->position.x()) + ", " + BURST::numeric::to_string(this->position.y()) + ") is not on the border of the configuration space. This may lead to unexpected movement behavior.";
                burst_warning(warning_string.c_str());
            }
        }

        /**
         * @brief Apply the rotation model to a commanded heading (no translation).
         * @return Perturbed angle.
         */
        numeric::fscalar perturb(const numeric::fscalar& angle) const {
            return this->rotation_model(angle);
        }

        /**
         * @brief Compute where the robot would stop if it moved along `angle` without updating state.
         *
         * When `perturbed` is true, the heading is first passed through the rotation model.
         *
         * @return `std::nullopt` if no configuration space is set or the motion is infeasible.
         */
        std::optional<geometry::Point2D> shootRay(const numeric::fscalar& angle, bool perturbed = false) const {
            // Cannot shoot ray if configuration environment does not exist
            if (!this->configuration_environment) return std::nullopt;
            return this->movement_model(this->position, perturbed ? this->rotation_model(angle) : angle, *this->configuration_environment);
        }
        /**
         * @brief Minkowski-style “stadium” swept by the disk along the feasible motion for `angle`.
         *
         * Unites start and end circular footprints with the connecting strip bounded by the
         * movement path type. Empty if the trajectory cannot be resolved.
         *
         * @return Covered region if the motion is feasible, `std::nullopt` otherwise.
         */
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
            return stadium;
        }

        /**
         * @brief Execute a motion: update @ref getPosition to the inward boundary hit, if any.
         *
         * @return `false` when no configuration space is set or the move is invalid.
         */
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

        /** 
         * @brief Default visualization color (red disk).
         * @return Default robot color.
         */
        renderable::Color defaultColor() const override {
            return renderable::Color{255, 0, 0};
        }

        /** @brief Draw the robot as a filled circle at the current position. */
        void render(renderable::Scene& scene, const renderable::Color& color) const override {
            if (this->radius <= 0) return; // If the robot has a non-positive radius, then there's nothing to render

            using arrangement_t = geometry::CurvilinearPolygonSet2D::Arrangement_2;
            using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, arrangement_t::Vertex_const_handle, arrangement_t::Halfedge_const_handle, arrangement_t::Face_const_handle>;
            // Render the robot as a circle at its current position filled with the specific color
            graphics_options_t robot_options;
            robot_options.colored_face = [](const arrangement_t&, const arrangement_t::Face_const_handle&) -> bool {
                return true;
            };
            robot_options.face_color = [color](const arrangement_t&, const arrangement_t::Face_const_handle&) -> renderable::Color {
                return color;
            };

            // Construct an empty arrangement to enter
            arrangement_t arrangement;
            // Insert the circle representing the robot's current position into the arrangement
            std::optional<geometry::CurvilinearPolygon2D> circle = geometry::construct_circle(this->radius, this->position);
            if (circle) CGAL::insert(arrangement, circle->curves_begin(), circle->curves_end());
            // This should never occur due to earlier checks, but log the error if it does anyway
            else burst_error("Failed to render robot due to non-positive radius.");
            
            CGAL::add_to_graphics_scene(arrangement, scene, robot_options);
        }
    };

}

#endif
