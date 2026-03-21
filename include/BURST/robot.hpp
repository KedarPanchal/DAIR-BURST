#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <memory>
#include <random>

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

    public:
        Robot(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error) : 
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{max_rotation_error}, 
            movement_model{} {}
        Robot(numeric::fscalar robot_radius, geometry::Point2D starting_point, numeric::fscalar max_rotation_error, unsigned int rotation_seed) : 
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{max_rotation_error, rotation_seed}, 
            movement_model{} {}
        Robot(numeric::fscalar robot_radius, geometry::Point2D starting_point, models::RotationModel<PRNG, Dist> rotation_model, models::MovementModel<Trajectory, Path> movement_model) : 
            radius{robot_radius}, 
            position{starting_point}, 
            rotation_model{rotation_model}, 
            movement_model{movement_model} {}
        
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

        std::optional<geometry::Point2D> shootRay(numeric::fscalar angle) const {
            auto trajectory = this->movement_model.path(this->position, angle, *this->configuration_environment);
            return trajectory.has_value() ? std::optional<geometry::Point2D>{trajectory->endpoint()} : std::nullopt;
        }
        geometry::Polygon2D generateStadium(numeric::fscalar angle) const;
        geometry::Polygon2D generateCCR(numeric::fscalar angle) const;
        void move(numeric::fscalar angle);

        void render(graphics::Scene& scene) const override;
    };

}

#endif
