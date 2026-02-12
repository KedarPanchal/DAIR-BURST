#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "models.hpp"

using ConfigurationGeometry = BURST::geometry::ConfigurationGeometry;
using RotationModel = BURST::models::RotationModel;
using MovementModel = BURST::models::MovementModel;

namespace BURST {

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    class Robot : public Renderable {
    private:
        fscalar radius;
        fscalar x_position;
        fscalar y_position;
        std::unique_ptr<ConfigurationGeometry> configuration_environment;

        std::unique_ptr<RotationModel> rotation_model;
        std::unique_ptr<MovementModel> movement_model;

    public:
        Robot(fscalar robot_radius, fscalar max_rotation_error);
        Robot(fscalar robot_radius, fscalar max_rotation_error, unsigned int rotation_seed);
        Robot(fscalar robot_radius, fscalar max_rotation_error, fscalar fixed_rotation_scale);
        Robot(fscalar robot_radius, std::unique_ptr<RotationModel> rotation_model, std::unique_ptr<MovementModel> movement_model);
        void setConfigurationEnvironment(std::unique_ptr<ConfigurationGeometry> config_environment);
        const ConfigurationGeometry& getConfigurationEnvironment() const;

        fscalar getRadius() const;
        Point_2 shootRay(fscalar angle) const;
        Polygon_2 generateStadium(fscalar angle) const;
        Polygon_2 generateCCR(fscalar angle) const;
        void move(fscalar angle);

        void render(scene& scene) const override;
    };

}
#endif
