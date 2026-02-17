#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "models.hpp"

namespace BURST {
    // Declare type traits for validating Robot class template parameters
    template <typename R>
    struct is_valid_rotation_model : std::false_type {};
    template <typename PRNG, typename Dist>
    struct is_valid_rotation_model<BURST::models::RotationModel<PRNG, Dist>> : std::true_type {};

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    template <typename R, typename M>
    class Robot : public Renderable {
    private:
        fscalar radius;
        fscalar x_position;
        fscalar y_position;
        std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_environment;

        std::unique_ptr<R> rotation_model;
        std::unique_ptr<M> movement_model;

    public:
        Robot(fscalar robot_radius, fscalar max_rotation_error);
        Robot(fscalar robot_radius, fscalar max_rotation_error, unsigned int rotation_seed);

        void setConfigurationEnvironment(std::unique_ptr<BURST::geometry::ConfigurationGeometry> config_environment);
        const BURST::geometry::ConfigurationGeometry& getConfigurationEnvironment() const;

        fscalar getRadius() const;
        Point_2 shootRay(fscalar angle) const;
        Polygon_2 generateStadium(fscalar angle) const;
        Polygon_2 generateCCR(fscalar angle) const;
        void move(fscalar angle);

        void render(scene& scene) const override;
    };

}
#endif
