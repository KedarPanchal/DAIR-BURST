#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "types.hpp"
#include "numeric_types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "models.hpp"

namespace BURST {

    // Internal implementations not intended for public use
    namespace detail {
        // Declare type traits for validating Robot class template parameters
        template <typename T>
        struct is_valid_rotation_model : std::false_type {};
        template <typename PRNG, typename Dist>
        struct is_valid_rotation_model<BURST::models::RotationModel<PRNG, Dist>> : std::true_type {};
    }

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    template <typename R, typename M>
    class Robot : public Renderable {
    // Validate type traits
    static_assert(detail::is_valid_rotation_model<R>::value, "R must be a valid rotation model");

    private:
        numeric::fscalar radius;
        numeric::fscalar x_position;
        numeric::fscalar y_position;
        std::unique_ptr<BURST::geometry::ConfigurationGeometry> configuration_environment;

        std::unique_ptr<R> rotation_model;
        std::unique_ptr<M> movement_model;

    public:
        Robot(numeric::fscalar robot_radius, numeric::fscalar max_rotation_error);
        Robot(numeric::fscalar robot_radius, numeric::fscalar max_rotation_error, unsigned int rotation_seed);

        void setConfigurationEnvironment(std::unique_ptr<BURST::geometry::ConfigurationGeometry> config_environment);
        const BURST::geometry::ConfigurationGeometry& getConfigurationEnvironment() const;

        numeric::fscalar getRadius() const;
        Point_2 shootRay(numeric::fscalar angle) const;
        Polygon_2 generateStadium(numeric::fscalar angle) const;
        Polygon_2 generateCCR(numeric::fscalar angle) const;
        void move(numeric::fscalar angle);

        void render(scene& scene) const override;
    };

}
#endif
