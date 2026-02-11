#ifndef CONFIGURATION_GEOMETRY_HPP
#define CONFIGURATION_GEOMETRY_HPP

#include <optional>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include "types.hpp"
#include "renderable.hpp"

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
