#ifndef CONFIGURATION_GEOMETRY_HPP
#define CONFIGURATION_GEOMETRY_HPP

#include <optional>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include "renderable.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using scene = CGAL::Graphics_scene;

namespace BURST::geometry {

    /*
     * ConfigurationGeometry represents the geometry of the robot's configuration space.
     */
    class ConfigurationGeometry : public Renderable {
    public:
        virtual std::optional<Segment_2> getEdge(Point_2 intersection_point) const = 0;
        virtual std::optional<Segment_2> getEdge(Segment_2 intersection_segment) const noexcept = 0;
        virtual void render(scene& scene) const override = 0;
    };
    
}
#endif
