#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <CGAL/Gmpq.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "models.hpp"

using precise_float = CGAL::Gmpq;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using scene = CGAL::Graphics_scene;

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
        const precise_float radius;
        precise_float x_position;
        precise_float y_position;
        std::unique_ptr<ConfigurationGeometry> configuration_environment;

        std::unique_ptr<RotationModel> rotation_model;
        std::unique_ptr<MovementModel> movement_model;

    public:
        Robot(precise_float robot_radius, precise_float max_rotation_error);
        Robot(precise_float robot_radius, precise_float max_rotation_error, unsigned int rotation_seed);
        Robot(precise_float robot_radius, precise_float max_rotation_error, precise_float fixed_rotation_scale);
        Robot(precise_float robot_radius, std::unique_ptr<RotationModel> rotation_model, std::unique_ptr<MovementModel> movement_model);
        void setConfigurationEnvironment(std::unique_ptr<ConfigurationGeometry> config_environment);

        precise_float getRadius() const;
        Point_2 shootRay(precise_float angle) const;
        Polygon_2 generateStadium(precise_float angle) const;
        Polygon_2 generateCCR(precise_float angle) const;
        void move(precise_float angle);

        void render(scene& scene) const override;
    };

}
#endif
