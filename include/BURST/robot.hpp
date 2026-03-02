#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <memory>

#include "numeric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "models.hpp"

namespace BURST {

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    template <models::valid_rotation_model R, typename M>
    class Robot : public Renderable {
    private:
        numeric::fscalar radius;
        numeric::fscalar x_position;
        numeric::fscalar y_position;
        std::unique_ptr<BURST::geometry::ConfigurationSpace> configuration_environment;

        std::unique_ptr<R> rotation_model;
        std::unique_ptr<M> movement_model;

    public:
        Robot(numeric::fscalar robot_radius, numeric::fscalar max_rotation_error);
        Robot(numeric::fscalar robot_radius, numeric::fscalar max_rotation_error, unsigned int rotation_seed);

        void setConfigurationEnvironment(std::unique_ptr<BURST::geometry::ConfigurationSpace> config_environment);
        const BURST::geometry::ConfigurationSpace& getConfigurationEnvironment() const;

        numeric::fscalar getRadius() const;
        geometry::Point2D shootRay(numeric::fscalar angle) const;
        geometry::Polygon2D generateStadium(numeric::fscalar angle) const;
        geometry::Polygon2D generateCCR(numeric::fscalar angle) const;
        void move(numeric::fscalar angle);

        void render(graphics::Scene& scene) const override;
    };

}
#endif
