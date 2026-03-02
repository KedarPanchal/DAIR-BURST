#ifndef BURST_ROBOT_HPP
#define BURST_ROBOT_HPP

#include <type_traits>
#include <memory>

#include "numeric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "models.hpp"

namespace BURST {

    // Internal implementations not intended for public use
    namespace detail {
        /*
         * Declare type traits to check if a type is a valid rotaition model
         * This means it must be an instantiation of the RotationModel template
         * Since the library is targeting C++20, this SFINAE needs to be used in tandem with concepts, since is_specialization_of is a C++23 feature
         */
        template <typename T>
        struct is_valid_rotation_model : std::false_type {};
        template <typename PRNG, typename Dist>
        struct is_valid_rotation_model<BURST::models::RotationModel<PRNG, Dist>> : std::true_type {};
    }
    
    // Checks if a type is a valid rotation model, as defined above, and wraps it as a concept
    template <typename R>
    concept valid_rotation_model = detail::is_valid_rotation_model<R>::value;

    /*
     * The robot class represents a circular, blind, unreliable robot.
     * Its rotational and translational movements are affected by noise and uses models to determine the impact of this noise.
     */
    template <valid_rotation_model R, typename M>
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
